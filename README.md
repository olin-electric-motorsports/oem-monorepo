<h1 align="center">
	<img
		alt="Olin Electric Motorsports"
		src="https://nyc3.digitaloceanspaces.com/oem-outline/logo-smaller.png">
</h1>

Welcome to the Olin Electric Motorsports monorepo. This is the home for all of our
electrical, firmware, and software work.

This document will help you get started contributing and walk you through
important steps for working on the team.



## Getting Started

If you are on a completely fresh Ubuntu computer, you won't have the tools needed to download this repository yet. Open your terminal (`Ctrl` + `Alt` + `T`) and run these commands one by one to install Git and clone our code:

**1. Install Git:**
```shell
sudo apt update
sudo apt install git -y
git clone https://github.com/olin-electric-motorsports/oem-monorepo oem-monorepo
cd oem-monorepo
```
Now we need to install a few things so lets run. 
```shell
./scripts/startup/quick-setup.sh
```

Go through this and install the packages



## Repository Configuration

Once your system has all the required packages, you need to configure your local repository settings to keep our KiCad libraries clean.
*   Run the KiCad Git filters script to apply rules that remove temporary files and prevent clutter:
    `./scripts/startup/install_kicad_git_filters.sh`
*   Run the KiCad Git Hooks script to install Git LFS and the hooks required to generate our symbol libraries on every push/pull:
    `./scripts/startup/install_kicad_git_hooks.sh`

## Testing Your Setup

Verify that your system is ready by running the following commands in your terminal:
*   Check your Bazel installation: `bazel version`
*   Check your OpenOCD installation: `openocd --version`
*   Test the vehicle firmware build: `bazel build //vehicle/... --config=m4`

If the final command outputs a lot of green text, your environment is perfectly configured!

## Firmware Workflow

Here are the standard Bazel commands you will use to interact with the STM32 microcontrollers. 

**Building specific targets (creates the .elf file):**
`bazel build --config=m4 //vehicle/examples/blinky:blinky.elf`

**Initializing new chips (configures the boot pin on fresh silicon):**
`bazel run --config=m4 //vehicle/examples/blinky:blinky_initialize`

**Flashing firmware using the ST-Link:**
`bazel run --config=m4 //vehicle/examples/blinky:blinky_flash`

**Debugging using OpenOCD and GDB:**
`bazel run -c dbg --config=m4 //vehicle/examples/blinky:blinky_debug`

## Contributing: Branches and Pull Requests

To keep our codebase stable and functional, we do not push code directly to the `main` branch. Instead, we use branches and Pull Requests (PRs) so code can be reviewed before it is merged. Here is the standard workflow for contributing:

**1. Get the Latest Code**
Always start by making sure your local repository is up-to-date with everyone else's work:
```shell
git checkout main
git pull origin main
```

**2. Create a New Branch**
Create a new branch for your specific feature or fix. Use a descriptive name and your name so the team knows what you are working on (e.g., `CoolerJacob/blinky-led`, `Jelly_Kelly/adc-reading`):
```shell
git checkout -b your-branch-name
```

**3. Make Your Changes and Commit**
Write your code, verify it builds, and then stage and commit your changes with a clear, descriptive message:
```shell
git add .
git commit -m "Brief description of what you changed and why"
```

**4. Push Your Branch**
Upload your newly created branch to the remote repository so the team can see it:
```shell
git push -u origin your-branch-name
```

**5. Open a Pull Request (PR)**
*   Go to our repository in your web browser.
*   You will usually see a green prompt saying "Compare & pull request" for your recently pushed branch.
*   Click it, fill out a description of what your code does, and submit the PR.
*   Tag a teammate to review your code. Once it is approved, you can merge it into `main`!