const path = require('path');
const CopyWebpackPlugin = require("copy-webpack-plugin");

module.exports = {
	devtool: 'eval-source-map',
	entry: "./src/index.ts",
	output: {
		path: path.resolve(__dirname, "dist"),
		filename: "index.js",
	},
	resolve: {
		extensions: [
			'.ts', '.js', // Allows importing from .ts and .js files.
		],
	},
	mode: "development",
	plugins: [
		new CopyWebpackPlugin({
			patterns: [
				{
					from: path.resolve(__dirname, "src", "index.html"),
					to: path.resolve(__dirname, "dist", "index.html")
				},
			],
		}),
	],
	module: {
		rules: [
			{
				test: /\.ts$/i,
				use: 'ts-loader',
				include: [path.resolve(__dirname, 'src')],
			}
		]
	},
	experiments: {
		asyncWebAssembly: true,
	},
};