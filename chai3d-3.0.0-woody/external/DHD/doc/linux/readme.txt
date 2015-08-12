Force Dimension devices installation instructions
-------------------------------------------------


In order to use the Force Dimension devices with Chai3d on Linux, a little bit of configuration is required. The following instructions have been tested on Ubuntu 13.04 (64-bit), but should apply to most recent linux distributions with minimal changes (if any).


1. allow unprivileged users access to Force Dimension devices (optional)

By default, linux requires superuser privileges for RAW USB access, which the Force Dimension devices requires. This can be changed by adding the appropriate rule to the linux dynamic device management (udev). The attached file '11-forcedimension.rules' contains the correct rules for all Force Dimension devices. To enable it, simply copy it (with superuser privileges) to /etc/udev/rules.d, and restart 'udev' (or reboot your machine).


________________________
(C) 2003-2014 by CHAI 3D
All Rights Reserved.

$Rev: 1297 $
