import async_wasm_three_body from 'wasm-three-body/wasm_three_body';
import async_wasm_three_body_bg from "wasm-three-body/wasm_three_body_bg.wasm"

async function main() {
	const wasm_three_body = await async_wasm_three_body;
	const wasm_three_body_bg  = await async_wasm_three_body_bg;
}

window.addEventListener("DOMContentLoaded", main, false);