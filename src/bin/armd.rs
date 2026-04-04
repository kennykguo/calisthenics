// ABOUTME: Runs a localhost TCP bridge that relays JSON arm commands to a persistent armctl serve child.
// ABOUTME: Keeps the STM32 serial session open so other local processes can issue request/response lines cheaply.

use std::{
    io::{BufRead, BufReader, Write},
    net::{TcpListener, TcpStream},
    path::{Path, PathBuf},
    process::{Child, ChildStdin, ChildStdout, Command as ProcessCommand, Stdio},
    sync::{Arc, Mutex},
    thread,
};

const DEFAULT_LISTEN_ADDR: &str = "127.0.0.1:7878";

#[derive(Clone, Debug, Eq, PartialEq)]
struct ServerConfig {
    device_path: String,
    listen_addr: String,
}

struct ArmBridge {
    _child: Child,
    child_stdin: ChildStdin,
    child_stdout: BufReader<ChildStdout>,
}

trait LineRelay {
    fn relay_line(&mut self, request_line: &str) -> Result<String, String>;
}

fn main() {
    if let Err(error) = run() {
        eprintln!("{error}");
        std::process::exit(1);
    }
}

fn run() -> Result<(), String> {
    let config = parse_config(std::env::args().skip(1).collect())?;
    let listener = TcpListener::bind(&config.listen_addr)
        .map_err(|error| format!("failed to bind {}: {error}", config.listen_addr))?;
    let armctl_path = armctl_binary_path(
        &std::env::current_exe()
            .map_err(|error| format!("failed to resolve current exe: {error}"))?,
    );
    let bridge = Arc::new(Mutex::new(ArmBridge::spawn(
        &armctl_path,
        &config.device_path,
    )?));

    for stream in listener.incoming() {
        let stream = stream.map_err(|error| format!("failed to accept connection: {error}"))?;
        let client_bridge = Arc::clone(&bridge);
        thread::spawn(move || {
            if let Err(error) = handle_client(stream, client_bridge) {
                eprintln!("{error}");
            }
        });
    }

    Ok(())
}

fn parse_config(args: Vec<String>) -> Result<ServerConfig, String> {
    match args.as_slice() {
        [device_path] => Ok(ServerConfig {
            device_path: device_path.clone(),
            listen_addr: String::from(DEFAULT_LISTEN_ADDR),
        }),
        [device_path, listen_addr] => Ok(ServerConfig {
            device_path: device_path.clone(),
            listen_addr: listen_addr.clone(),
        }),
        _ => Err(usage()),
    }
}

fn usage() -> String {
    String::from("usage: cargo run --bin armd -- <serial-device> [listen-addr]")
}

fn armctl_binary_path(current_exe: &Path) -> PathBuf {
    current_exe.with_file_name("armctl")
}

impl ArmBridge {
    fn spawn(armctl_path: &Path, device_path: &str) -> Result<Self, String> {
        let mut child = ProcessCommand::new(armctl_path)
            .arg(device_path)
            .arg("serve")
            .arg("--json")
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .spawn()
            .map_err(|error| format!("failed to spawn {}: {error}", armctl_path.display()))?;

        let child_stdin = child
            .stdin
            .take()
            .ok_or_else(|| String::from("armctl child stdin was not piped"))?;
        let child_stdout = child
            .stdout
            .take()
            .ok_or_else(|| String::from("armctl child stdout was not piped"))?;

        Ok(Self {
            _child: child,
            child_stdin,
            child_stdout: BufReader::new(child_stdout),
        })
    }

    fn relay_line(&mut self, request_line: &str) -> Result<String, String> {
        self.child_stdin
            .write_all(request_line.as_bytes())
            .map_err(|error| format!("failed to write request to armctl child: {error}"))?;
        self.child_stdin
            .flush()
            .map_err(|error| format!("failed to flush request to armctl child: {error}"))?;

        let mut response_line = String::new();
        let bytes_read = self
            .child_stdout
            .read_line(&mut response_line)
            .map_err(|error| format!("failed to read response from armctl child: {error}"))?;

        if bytes_read == 0 {
            return Err(String::from("armctl child closed stdout"));
        }

        Ok(response_line)
    }
}

impl LineRelay for ArmBridge {
    fn relay_line(&mut self, request_line: &str) -> Result<String, String> {
        ArmBridge::relay_line(self, request_line)
    }
}

fn handle_client<R>(mut stream: TcpStream, bridge: Arc<Mutex<R>>) -> Result<(), String>
where
    R: LineRelay + Send + 'static,
{
    let mut reader = BufReader::new(
        stream
            .try_clone()
            .map_err(|error| format!("failed to clone client stream: {error}"))?,
    );

    loop {
        let mut request_line = String::new();
        let bytes_read = reader
            .read_line(&mut request_line)
            .map_err(|error| format!("failed to read client request: {error}"))?;

        if bytes_read == 0 {
            return Ok(());
        }

        if !request_line.ends_with('\n') {
            request_line.push('\n');
        }

        let response_line = bridge
            .lock()
            .map_err(|_| String::from("arm bridge lock poisoned"))?
            .relay_line(&request_line)?;
        stream
            .write_all(response_line.as_bytes())
            .map_err(|error| format!("failed to write client response: {error}"))?;
        stream
            .flush()
            .map_err(|error| format!("failed to flush client response: {error}"))?;
    }
}

#[cfg(test)]
mod tests {
    use super::{
        DEFAULT_LISTEN_ADDR, LineRelay, ServerConfig, armctl_binary_path, handle_client,
        parse_config,
    };
    use std::{
        collections::VecDeque,
        io::{BufRead, BufReader, Write},
        net::{TcpListener, TcpStream},
        path::Path,
        sync::{Arc, Mutex},
        thread,
    };

    struct FakeRelay {
        requests: Vec<String>,
        responses: VecDeque<String>,
    }

    impl FakeRelay {
        fn new(responses: &[&str]) -> Self {
            Self {
                requests: Vec::new(),
                responses: responses
                    .iter()
                    .map(|response| String::from(*response))
                    .collect(),
            }
        }
    }

    impl LineRelay for FakeRelay {
        fn relay_line(&mut self, request_line: &str) -> Result<String, String> {
            self.requests.push(String::from(request_line));
            self.responses
                .pop_front()
                .ok_or_else(|| String::from("no fake relay response available"))
        }
    }

    fn strings(values: &[&str]) -> Vec<String> {
        values.iter().map(|value| String::from(*value)).collect()
    }

    #[test]
    fn uses_the_default_listen_address() {
        assert_eq!(
            parse_config(strings(&["/dev/ttyACM0"])),
            Ok(ServerConfig {
                device_path: String::from("/dev/ttyACM0"),
                listen_addr: String::from(DEFAULT_LISTEN_ADDR),
            })
        );
    }

    #[test]
    fn accepts_a_custom_listen_address() {
        assert_eq!(
            parse_config(strings(&["/dev/ttyACM0", "127.0.0.1:9000"])),
            Ok(ServerConfig {
                device_path: String::from("/dev/ttyACM0"),
                listen_addr: String::from("127.0.0.1:9000"),
            })
        );
    }

    #[test]
    fn resolves_the_sibling_armctl_binary() {
        assert_eq!(
            armctl_binary_path(Path::new("/tmp/target/debug/armd")),
            Path::new("/tmp/target/debug/armctl")
        );
    }

    #[test]
    fn relays_lines_for_a_single_client() {
        let listener = TcpListener::bind("127.0.0.1:0").expect("test listener should bind");
        let address = listener
            .local_addr()
            .expect("test listener should publish an address");
        let relay = Arc::new(Mutex::new(FakeRelay::new(&["{\"kind\":\"ok\"}\n"])));
        let server_relay = Arc::clone(&relay);

        let server_thread = thread::spawn(move || {
            let (stream, _) = listener.accept().expect("test server should accept");
            handle_client(stream, server_relay).expect("client relay should succeed");
        });

        let mut client = TcpStream::connect(address).expect("test client should connect");
        client
            .write_all(b"{\"command\":\"status\"}\n")
            .expect("test client should write a request");
        client
            .shutdown(std::net::Shutdown::Write)
            .expect("test client should close its write side");

        let mut reader = BufReader::new(client);
        let mut response = String::new();
        reader
            .read_line(&mut response)
            .expect("test client should read a response");

        server_thread
            .join()
            .expect("test server thread should exit cleanly");

        assert_eq!(response, "{\"kind\":\"ok\"}\n");
        assert_eq!(
            relay
                .lock()
                .expect("fake relay should stay unlocked")
                .requests,
            vec![String::from("{\"command\":\"status\"}\n")]
        );
    }

    #[test]
    fn relays_lines_for_multiple_clients_with_a_shared_bridge() {
        let listener = TcpListener::bind("127.0.0.1:0").expect("test listener should bind");
        let address = listener
            .local_addr()
            .expect("test listener should publish an address");
        let relay = Arc::new(Mutex::new(FakeRelay::new(&[
            "{\"id\":\"req-1\",\"kind\":\"ok\"}\n",
            "{\"id\":\"req-2\",\"kind\":\"status\"}\n",
        ])));
        let server_relay = Arc::clone(&relay);

        let server_thread = thread::spawn(move || {
            let mut client_threads = Vec::new();
            for _ in 0..2 {
                let (stream, _) = listener.accept().expect("test server should accept");
                let client_relay = Arc::clone(&server_relay);
                client_threads.push(thread::spawn(move || {
                    handle_client(stream, client_relay).expect("client relay should succeed");
                }));
            }

            for client_thread in client_threads {
                client_thread
                    .join()
                    .expect("per-client server thread should exit cleanly");
            }
        });

        let mut first_client = TcpStream::connect(address).expect("first client should connect");
        first_client
            .write_all(b"{\"id\":\"req-1\",\"command\":\"clear-fault\"}\n")
            .expect("first client should write a request");
        first_client
            .shutdown(std::net::Shutdown::Write)
            .expect("first client should close its write side");
        let mut first_reader = BufReader::new(first_client);
        let mut first_response = String::new();
        first_reader
            .read_line(&mut first_response)
            .expect("first client should read a response");

        let mut second_client = TcpStream::connect(address).expect("second client should connect");
        second_client
            .write_all(b"{\"id\":\"req-2\",\"command\":\"status\"}\n")
            .expect("second client should write a request");
        second_client
            .shutdown(std::net::Shutdown::Write)
            .expect("second client should close its write side");
        let mut second_reader = BufReader::new(second_client);
        let mut second_response = String::new();
        second_reader
            .read_line(&mut second_response)
            .expect("second client should read a response");

        server_thread
            .join()
            .expect("test server thread should exit cleanly");

        assert_eq!(first_response, "{\"id\":\"req-1\",\"kind\":\"ok\"}\n");
        assert_eq!(second_response, "{\"id\":\"req-2\",\"kind\":\"status\"}\n");
        assert_eq!(
            relay
                .lock()
                .expect("fake relay should stay unlocked")
                .requests,
            vec![
                String::from("{\"id\":\"req-1\",\"command\":\"clear-fault\"}\n"),
                String::from("{\"id\":\"req-2\",\"command\":\"status\"}\n"),
            ]
        );
    }
}
