dVRK configuration files for JHU
================================

Since we have multiple systems at JHU, the configuration files are
organized in sub-directories.  Directory names start with the
institution name (e.g. jhu for Johns Hopkins, isi for Intuitive
Surgical) and should contain the system name (e.g. JHU has four
systems, a Classic research kit: `jhu-dVRK`, a full da Vinci Classic:
`jhu-daVinci`, a full da Vinci Si: `jhu-daVinci-Si` and a Si Kit:
`jhu-dVRK-Si`).

We strongly encourage each dVRK site to create their own repository
based on this one.  If you want to add this to your ROS workspace,
don't forget to edit the `CMakeLists.txt` and `package.xml` to rename
the package (query replace the `dvrkconfig_jhu` with `dvrk_config_XXX`
(`XXX` being your institution's acronym). Once this is done and after
you rebuilt your workspace, you will be able to do `roscd
dvrk_config_jhu` or use this path to locate the dVRK configuration
files in your own launch files.

Each directory should contain:
  * your IO configuration files, `sawRobotIO1394-xxxxx.xml`, for each arm identified by its serial
number.  For Classic arms, you might also want to store the original `.cal` files provided by Intuitive Surgical since they are needed to re-generate the IO XML files.
  * your arms configuration files (for Classic systems)
  * your console configuration files since these refer to your system specific IO configuration files
