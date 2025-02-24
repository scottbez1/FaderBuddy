Import("env")

# List installed packages
env.Execute("$PYTHONEXE -m pip list")

# Install custom packages from the PyPi registry
env.Execute("$PYTHONEXE -m pip install https://github.com/mraardvark/pyupdi/archive/master.zip")
