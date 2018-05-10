# Bare Minimum

This project is an example of the absolute minimum required to get a Tomu project running.

The code first disables the watchdog, which is required to prevent Tomu from restarting immediately.  Then it loops forever.

The Makefile takes care of compiling everything and linking it all, as well as determining dependencies.  That is, if you modify a header file, then all files that depend on it will be rebuilt.

The resulting .dfu file is based on the directory name.