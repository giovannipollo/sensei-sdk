# SENSEI SDK (Smart Embedded Nano SEnsor Interface)

Software development kit for the SENSEI ecosystem.

This repository contains the SENSEI specific SDK for the NRF5340 and the GAP9.

**Currently the nRF Connect SDK v2.6.1 is supported!**

## Getting Started

First, install the NRF Connect SDK and GAP9 SDK as described in [Install.md](Install.md).

Next, clone this repository:

```sh
git clone git@iis-git.ee.ethz.ch:sensei/software/sensei-sdk.git
cd sensei-sdk
git submodule update --init --recursive
```

Install pre-commit hooks.

```sh
source scripts/install-hooks.sh

```

Make sure to export the following environment variables.

```sh
export SENSEI_SDK_ROOT=$(pwd)

# Bash (Permanently)
echo "export SENSEI_SDK_ROOT=$(pwd)" >> ~/.bashrc

# Zsh (Permanently)
echo "export SENSEI_SDK_ROOT=$(pwd)" >> ~/.zshrc

# Fish (Permanently)
echo "set -x SENSEI_SDK_ROOT $(pwd)" >> ~/.config/fish/config.fish
```

## Examples
Each example contains two folders: `src_GAP9` and `src_NRF`. The former contains the GAP9 specific code, while the latter contains the NRF specific code.
First, flash the NRF application by opening it in Visual Studio Code using the NRF Connect SDK extension or use the terminal as described below.
**Make sure your `SENSEI_SDK_ROOT` environment variable is set to the path of the SENSEI SDK!**


### nRF: Use Visual Studio Code
Open the `sensei-sdk.code-workspace` file in Visual Studio Code. This will load the workspace with all relevant folders.
Furthermore, it will automatically set the the SENSEI board to to list of supported boards in the nRF Connect SDK extension.

1. Use the file explorer to navigate to the desired example folder and select "nRF Connect: Add Folder as Application" on the `src_NRF` folder.
2. In the "Applications" tab of the nRF Connect SDK extension, select the newly added application and click on "Add build configuration".
3. In the new window under "CMakePreset", select "Build for NRF5340 SENSEIV1C APP (build)", and select `v2.6.1` as SDK version and `v2.9.1` as Toolchain version.
4. Click on "Generate and Build" to build the application.
5. In the "Actions" tab click on "Flash" to flash the application to the board.

### nRF: Use Terminal
Make sure to export `ZEPHYR_BASE` pointing to the correct nRF Connect SDK folder and you activated the mamba/conda environment as described in [Install.md](Install.md).

To build and flash the nRF application, open a terminal in the `src_NRF` folder of the desired example and run the following commands:
```sh
cd <example>/src_NRF
west build -b nrf5340_senseiv1_cpuapp
west flash
```

### GAP9: Use Terminal
Next, flash the GAP9 application by opening a terminal in the `src_GAP9` folder and running the following command:

```sh
gap init
gap run
```

In case you want to use GVSoC, modify the `sdk.config` and enable `CONFIG_PLATFORM_GVSOC=y`. Alternatively, run:

```sh
gap menuconfig
```
to configure the configuration with a graphical interface in the terminal.


## BareMetal Drivers
- **MAX77654**: Analog Devices Power Management IC,


## Zephyr Divers
- **PCA6416A**: NXP GPIO Expander,
  - Requires `CONFIG_GPIO` and `CONFIG_GPIO_PCA64XXA` to be enabled.
- **IS31FL3194**: I2C RGB LED,
  - From *main* Zephyr branch (see [Docs](https://docs.zephyrproject.org/latest/build/dts/api/bindings/led/issi%2Cis31fl3194.html#std-dtcompatible-issi-is31fl3194))


## Maintainers
- **Philip Wiese** ([wiesep@iis.ee.ethz.ch](mailto:wiesep@iis.ee.ethz.ch))
- **Sebastian Frey** ([sefrey@iis.ee.ethz.ch](mailto:sefrey@iis.ee.ethz.ch))

## Formatting and Linting

We provide the [pre-commit](https://pre-commit.com) configuration file which you can use to install github hooks that execute the formatting commands on your changes.

You will need to manually install pre-commit since it's not added as a dependency to the `pyproject.toml`:
```bash
pip install pre-commit
```

The configuration sets the default stage for all the hooks to `pre-push` so to install the git hooks run:
```bash
pre-commit install --hook-type pre-push
```
The hooks will run before each push, making sure the pushed code can pass linting checks and not fail the CI on linting.

If you change your mind and don't want the git hooks:
```bash
pre-commit uninstall
```

## License
All licenses used in this repository are listed under the `LICENSES` folder. Unless specified otherwise in the respective file headers, all code checked into this repository is made available under a permissive license.
- Most software sources and tool scripts are licensed under the [Apache 2.0 license](https://opensource.org/licenses/Apache-2.0).
- Some third party files under the `NRF/system/third_party` directory are licensed under the [MIT license](https://opensource.org/license/mit).
- Some third party files under the `NRF/system/third_party` directory are licensed under the [BSD 3 license](https://opensource.org/license/bsd-3-clause).
- Markdown, JSON, text files, pictures, PDFs, are licensed under the [Creative Commons Attribution 4.0 International](https://creativecommons.org/licenses/by/4.0) license (CC BY 4.0).

To extract license information for all files, you can use the [reuse tool](https://reuse.software/) and by running `reuse spdx` in the root directory of this repository.
