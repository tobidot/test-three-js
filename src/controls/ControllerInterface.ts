interface ControllerInterface {
    update(): void;

    is_moving_forward: boolean;
    is_rotating_left: number;
    is_rotating_right: number;
}