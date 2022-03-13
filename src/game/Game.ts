import * as THREE from "three";
import { AssetLoaderDummyLevel } from "~/assets/AssetLoaderDummyLevel";
import AssetLoaderInterface from "~/assets/AssetLoaderInterface";
import * as CANNON from "cannon-es";
import { throw_expression } from "@game.object/ts-game-toolbox";

export class Game {
    public physics: CANNON.World;
    public scene: THREE.Scene;
    public camera: THREE.Camera;
    public renderer: THREE.WebGLRenderer;
    public assets: AssetLoaderInterface;
    public resolve_game: null | ((value: any) => void) = null;
    public reject_game: null | ((value: any) => void) = null;
    public last_animation_frame: number = 0;

    // test
    public sphere_body: CANNON.Body;
    public sphere_graphics: THREE.Object3D | null = null;

    public constructor() {
        // setup graphics
        this.renderer = new THREE.WebGLRenderer();
        this.renderer.setSize(window.innerWidth, window.innerHeight);
        document.body.appendChild(this.renderer.domElement);
        this.scene = new THREE.Scene();
        this.camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 0.1, 1000);
        // setup phyics
        this.physics = new CANNON.World();
        this.physics.gravity.set(0, -9.81, 0);
        // setup assets
        this.assets = new AssetLoaderDummyLevel(this.scene, this.camera);

        /**
         * Should be done on load
         */

        // Create a sphere
        const radius = 1; // m
        this.sphere_body = new CANNON.Body({
            mass: 0.25, // kg
            position: new CANNON.Vec3(0, 20.5, 5), // m
            shape: new CANNON.Box(new CANNON.Vec3(0.1, 0.1, 0.1)),
        });
        this.physics.addBody(this.sphere_body);

        // Create a plane
        const groundBody = new CANNON.Body({
            type: CANNON.Body.STATIC,
            mass: 0, // mass == 0 makes the body static       
            shape: new CANNON.Plane(),
            // position: new CANNON.Vec3(0, 0, 0),
        });
        groundBody.quaternion.setFromEuler(-Math.PI / 2, 0, 0)
        this.physics.addBody(groundBody);



        // const geometry = new THREE.BoxGeometry();
        // const material = new THREE.MeshBasicMaterial({ color: 0x00ff00 });
        // const cube = new THREE.Mesh(geometry, material);
        // scene.add(cube);



    }

    public async load() {
        return this.assets.load((progress) => {
            console.log(progress);
        }).then((assets) => {
            // read scene from assets
            const scene = assets.get('scene');
            if (scene instanceof THREE.Scene) {
                this.scene = scene;
                const cameras = scene.children.filter((value): value is THREE.PerspectiveCamera => {
                    return value instanceof THREE.PerspectiveCamera;
                });
                if (cameras.length > 0) {
                    this.camera = cameras[0];
                }
                this.sphere_graphics = scene.children.find((object) => {
                    return object.name === "Box";
                }) ?? throw_expression("Could not find box in scene");

                {
                    // const body = this.sphere_body;
                    // const mesh = this.sphere_graphics;
                    // body.position.x = mesh.position.x;
                    // body.position.y = mesh.position.y;
                    // body.position.z = mesh.position.z;
                    // body.quaternion.x = mesh.quaternion.x;
                    // body.quaternion.y = mesh.quaternion.y;
                    // body.quaternion.z = mesh.quaternion.z;
                    // body.quaternion.w = mesh.quaternion.w;
                }

                // const light = new THREE.AmbientLight(0x202020, 0.2); // soft white light
                // scene.add(light);
                const light2 = new THREE.SpotLight(0x808080, 1, 200); // soft white light
                light2.position.set(0, 200, 0);
                scene.add(light2);
            }
        });
    }

    public async run() {
        return new Promise((resolve, reject) => {
            this.resolve_game = resolve;
            this.reject_game = reject;
            this.on_animation_frame(performance.now());
        });
    }

    protected on_animation_frame = (time: number) => {
        const fixedTimeStep = 1.0 / 60.0; // seconds
        const maxSubSteps = 3;
        const delta_seconds = (time - this.last_animation_frame) / 1000.0;
        try {
            requestAnimationFrame(this.on_animation_frame);
            if (this.sphere_graphics) {
                this.physics.fixedStep(delta_seconds, maxSubSteps);
                const body = this.sphere_body;
                const mesh = this.sphere_graphics;
                console.log(
                    "hi",
                    mesh.position,
                    body.position,
                );
                mesh.position.x = body.position.x;
                mesh.position.y = body.position.y;
                // mesh.position.z = body.position.z;
                mesh.quaternion.x = body.quaternion.x;
                mesh.quaternion.y = body.quaternion.y;
                mesh.quaternion.z = body.quaternion.z;
                mesh.quaternion.w = body.quaternion.w;
            }
            this.renderer.render(this.scene, this.camera);
            this.last_animation_frame = time;
        } catch (error) {
            if (!this.reject_game) throw new Error("resolver functions not found");
            this.reject_game(error);
        }
    }

}