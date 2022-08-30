## VSCode tasks

In VSCode pressing `ctrl + shift + p` opens the command pallate, then type `Tasks: Run Task` to open the list of possible tasks to run. These are all defined in the `.vscode/tasks.json` file. They are the following:
- `Core` - Starts roscore so that programs can message eachother.
- `Build` - Builds all of the packages in the workspace.
- `Run` - Run an executable in one of the packages in the project.
- `Clean` - Deletes all build files.

## Creating new packages

Each package is a component of the robot software that runs independently of one another. When the `catkin build` is run, all packages in the `src` directory are built together. To create a new package, run `cakin create pkg package_name` in the `src` directory where `package_name` is the name of the new package. This will create a `CMakeLists.txt` and `package.xml` file that defines the configuration and build procedure for the package. Create a subfolder called `scripts` for python code in the package.
