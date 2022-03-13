import { Game } from "./game/Game";


const game = new Game();
game.load().then(() => {
    return game.run();
}).then(() => {
    console.log("game will close now");
});

declare module oimo {

}