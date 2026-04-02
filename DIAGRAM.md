                    +--------------------------------------+
                    |              jetson orin nano        |
                    |--------------------------------------|
camera ------------>| vision pipeline                      |
                    | object / gripper state estimator     |
                    | grasp proposal / policy              |
                    | world model / rl / planner           |
                    | safety-aware command generator       |
                    | telemetry logger + dataset writer    |
                    +-------------------+------------------+
                                        |
                                        | typed command protocol
                                        | primary: 3.3v uart
                                        | optional debug: usb serial
                                        v
                    +--------------------------------------+
                    |              stm32f446               |
                    |--------------------------------------|
                    | parser + command validator           |
                    | watchdog / heartbeat                 |
                    | arm state machine                    |
                    | joint limit clamp                    |
                    | rate / accel limiting                |
                    | one-joint-at-a-time scheduler        |
                    | brownout / fault handling            |
                    | pwm generation for 4 servos          |
                    +----+-----------+-----------+---------+
                         |           |           |
                         v           v           v
                    base servo   shoulder     elbow      gripper
                                               servo       servo

                    feedback path:
                    stm32 -> joint targets, fault flags, timing, motion status -> jetson