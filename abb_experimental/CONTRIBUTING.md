ROS-Industrial is a community project. We welcome contributions from any source, from those who are extremely active to casual users. The following sections outline the steps on how to contribute to ROS-Industrial. It assumes there is an existing repository to which one would like to contribute (item 1 in the figure above) and one is familiar with the Git "Fork and Branch" workflow, detailed [here](http://blog.scottlowe.org/2015/01/27/using-fork-branch-git-workflow/).

1. Before any development is undertaken, a contributor would communicate a need and/or issue to the ROS-Industrial community. This can be done by submitting an issue on the appropriate GitHub repo, the [issues repo](https://github.com/ros-industrial/ros_industrial_issues), or by posting a message in the [ROS-Industrial category on ROS Discourse](//swri-ros-pkg-dev@googlegroups.com). . Doing so may save you time if similar development is underway and ensure that whatever approach you take is acceptable to the community of reviewers once it is submitted.
2. The second step (item 2) is to implement your change. If you are working on a code contribution, we highly recommend you utilize the [ROS Qt-Creator Plug-in](http://rosindustrial.org/news/2016/6/9/ros-qt-ide-plugin). Verify that your change successfully builds and passes all tests.
3. Next, push your changes to a "feature" branch in your personal fork of the repo and issue a pull request (PR)(item 3). The PR allows maintainers to review the submitted code. Before the PR can be accepted, the maintainer and contributor must agree that the contribution is implemented appropriately. This process can take several back-and-forth steps (see [example](https://github.com/ros-industrial/motoman/pull/89)). Contributors should expect to spend as much time reviewing/changing the code as on the initial implementation. This time can be minimized by communicating with the ROS-Industrial community before any contribution is made.
4. Issuing a Pull Request (PR) triggers the [Travis Continuous Integrations (CI)](https://github.com/ros-industrial/industrial_ci) step (item 4) which happens automatically in the background. The Travis CI performs several operations, and if any of the steps below fail, then the PR is marked accordingly for the maintainer.
  * Travis Workflow:
    * Installs a barebones ROS distribution on a fresh Ubuntu virtual machine.
    * Creates a catkin workspace and puts the repository in it.
    * Uses wstool to check out any from-source dependencies (i.e. other repositories).
    * Resolves package dependencies using rosdep (i.e. install packages using apt-get).
    * Compiles the catkin workspace.
    * Runs all available unit tests.
5. If the PR passes Travis CI and one of the maintainers is satisfied with the changes, they post a +1 as a comment on the PR (item 5). The +1 signifies that the PR is ready to be merged. All PRs require at least one +1 and pass Travis CI before it can be merged.
6. The next step (item 6) is for the PR to be merged into the main branch. This is done through the GitHub web interface by selecting the “Merge pull request” button. After the PR is merged, all status badges are updated automatically.
7. Periodically, the maintainer will release the package (item 7), which then gets sent to the [ROS Build Farm](http://wiki.ros.org/build.ros.org) for Debian creation.
8. The publishing of the released packages (item 8) is managed by OSRF and is not on a set schedule. This usually happens when all packages for a given distro are built successfully and stable. The current status for the distro kinetic can be found [here](http://repositories.ros.org/status_page/ros_kinetic_default.html) . Navigating to other distros can be done by changing the distro name in the link.
9. Once the package has been published, it is available to be installed by the developer (item 9).
10. After the install of a new version, the developer may have questions, experience issues or it may not have the necessary functionality which should all be reported on the packages GitHub repository as an issue (item 10). If an issue is identified or there is missing functionality that the developer requires, the cycle starts back at (item 2).

For more details, please refer to the [ROS-I wiki](http://wiki.ros.org/Industrial/DevProcess).

## Licensing
Any contribution that you make to this repository will
be under the Apache 2 License, as dictated by that
[license](http://www.apache.org/licenses/LICENSE-2.0.html):

~~~
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
~~~