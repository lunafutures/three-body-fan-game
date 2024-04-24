const path = require('path');
const CopyWebpackPlugin = require("copy-webpack-plugin");

module.exports = {
	entry: "./src/bootstrap.js",
	output: {
		path: path.resolve(__dirname, "dist"),
		filename: "bootstrap.js",
	},
	mode: "development",
	plugins: [
		new CopyWebpackPlugin(['src/index.html']),
	],
};