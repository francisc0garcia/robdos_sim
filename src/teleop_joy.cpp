#include "teleop_joy.hpp"

ControlStateMachine::ControlStateMachine() {

    statePub = nh.advertise<robdos_sim::StateCommand>("teleop/stateMachine", 1);
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig) {
    (void) sig;
    tcsetattr(kfd, TCSANOW, &cooked);
    ros::shutdown();
    exit(0);
}

void ControlStateMachine::keyLoop() {
    char c;
    bool dirty;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts("Reading from keyboard");
    puts("---------------------------");
    puts("Use arrow keys to move between the states.");

    for (;;) {
        // get the next event from the keyboard
        if (read(kfd, &c, 1) < 0) {
            perror("read():");
            exit(-1);
        }

        int linear = 0;
        ROS_DEBUG("value: 0x%02X\n", c);

        dirty = true;

        switch (c) {
            case KEYCODE_I:
                ROS_DEBUG("INIT");
                state.cmd = state.INIT;
                break;
            case KEYCODE_R:
                ROS_DEBUG("REPOSO");
                state.cmd = state.REPOSO;
                break;
            case KEYCODE_S:
                ROS_DEBUG("SAFETY");
                state.cmd = state.SAFETY;
                break;
            case KEYCODE_W:
                ROS_DEBUG("RESET");
                state.cmd = state.RESET;
                break;
            case KEYCODE_T:
                ROS_DEBUG("TELEOPERATION");
                linear = 1;
                state.cmd = state.TELEOPERATION;
                break;
            case KEYCODE_A:
                ROS_DEBUG("AUTONOMOUS");
                state.cmd = state.AUTONOMOUS;
                break;
            case KEYCODE_P:
                ROS_DEBUG("SEMIAUTONOMOUS");
                state.cmd = state.SEMIAUTONOMOUS;
                break;
            case KEYCODE_C:
                ROS_DEBUG("CHANGE");
                state.cmd = state.CHANGE;
                break;
            case KEYCODE_O:
                ROS_DEBUG("SUCCEEDED");
                state.cmd = state.SUCCEEDED;
                break;
            default:
                ROS_DEBUG("DEFAULT");
                dirty = false;
                break;
        }

        if (dirty) {
            statePub.publish(state);
        }
    }
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "teleop_joy");
    ControlStateMachine control_state_machine;

    signal(SIGINT, quit);

    control_state_machine.keyLoop();

    return (0);
}
