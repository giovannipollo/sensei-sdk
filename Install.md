# NRF SDK Installation
Follow the instructions in the video tutorial [nRF Connect for VS Code](https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-VS-Code/Tutorials) or the written instruction about [Installing the nRF Connect SDK](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/installation/install_ncs.html).

Next make sure to install the required version of the toolchain and nRF SDK in VSCode. We recommend to use nRF Connect SDK version `v2.6.1` and the corresponding toolchain version `2.9.1`. You can check out the [Installing additional SDK versions](https://docs.nordicsemi.com/bundle/nrf-connect-vscode/page/get_started/quick_setup.html#installing-additional-sdk-versions) for more information.

Finally, you also need to install the `nrfjprog` command line tools. You can find them [here](https://www.nordicsemi.com/Products/Development-tools/nRF-Command-Line-Tools). We recommend to version `10.23.5`


Table with tested versions:
| nRF Connect SDK Version | Toolchain Version | nrfjprog Version |
|------------------------|-------------------|-------------------|
| v2.6.1                 | 2.6.1             | 10.23.5           |
| v2.6.1                 | 2.9.1             | 10.23.5           |

## Using the Terminal
In case you want to use the terminal instead of the VSCode extension, follow the instructions below.

Make sure to export `ZEPHYR_BASE` pointing to the correct nRF Connect SDK folder. For example

```bash
export ZEPHYR_BASE=/opt/nordic/ncs/v2.6.1/zephyr
```

Next we recommend to setup a mamba/conda environment with the following command:
```bash
mamba create -n nrf python=3.12
mamba activate nrf
mamba install cmake=3 ccache
```

Then install the required packages:
```bash
pip install -r $ZEPHYR_BASE/scripts/requirements.txt
```

# Install the GAP SDK
Follow the instruction on the [GAP SDK GitHub repository](https://github.com/IIS-PBL-wizards-on-GAP9/gap_sdk_private)

