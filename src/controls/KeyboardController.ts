export default class KeyboardController implements ControllerInterface {
    public is_moving_forward: boolean = false;
    public is_rotating_left: number = 0;
    public is_rotating_right: number = 0;

    constructor() {
        document.addEventListener('keydown', (event) => {
            switch (event.code) {
                case "KeyW":
                    this.is_moving_forward = true;
                    break;
                case "KeyA":
                    this.is_rotating_left = -1;
                    break;
                case "KeyD":
                    this.is_rotating_right = 1;
                    break;
            }
        });
        document.addEventListener('keyup', (event) => {
            switch (event.code) {
                case "KeyW":
                    this.is_moving_forward = false;
                    break;
                case "KeyA":
                    this.is_rotating_left = 0;
                    break;
                case "KeyD":
                    this.is_rotating_right = 0;
                    break;
            }
        });
    }

    public update() {

    }
}