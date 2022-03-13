import * as THREE from "three";
import AssetLoaderInterface from "./AssetLoaderInterface";
/**
 * Loads the assets for the dummy level into the given scene and camera
 */
export class AssetLoaderDummyLevel implements AssetLoaderInterface {
    protected loaded: number = 0;
    protected total: number = 0;

    constructor(
        protected scene: THREE.Scene,
        protected camera: THREE.Camera
    ) {

    }

    public async load(
        on_update: (progress: number, total: number) => void
    ): Promise<Map<string, any>> {
        const promises = ([
            ["scene", (new THREE.ObjectLoader).loadAsync("/levels/scene.json")],
        ] as const).map(([key, promise]) => {
            return promise.then((value): [string, any] => [key, value]);
        });
        this.total = promises.length;
        promises.forEach((promise) => {
            promise.then((value) => {
                this.loaded++;
                on_update(this.loaded / this.total, this.total);
                return value;
            })
        });
        const assets = await (Promise.all(promises)).catch(
            (error) => {
                console.log("Failed to load assets");
                return [];
            }
        );
        return new Map(assets);
    }
}