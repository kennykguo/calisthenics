# ABOUTME: Builds and flashes the STM32F446RE firmware image through an ST-LINK probe with OpenOCD.
# ABOUTME: Uses the release embedded binary so hardware bring-up starts from a repeatable command.

set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
firmware_path="$repo_root/target/thumbv7em-none-eabihf/release/stm32f446re"

. "$HOME/.cargo/env"

cd "$repo_root"
cargo build-embedded

openocd \
  -f interface/stlink.cfg \
  -f target/stm32f4x.cfg \
  -c "program $firmware_path verify reset exit"
