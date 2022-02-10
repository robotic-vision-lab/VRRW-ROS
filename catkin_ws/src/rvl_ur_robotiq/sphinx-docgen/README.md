# Sphinx Documentation Generator

## Requirements

Since the documentation is ROS dependant, it must be build within the container after all ROS modules
are built and sourced properly.

Initial setup includes install [Sphinx](https://www.sphinx-doc.org/en/master/) and some of it's plugins for Markdown support.

```console
$ pip install -r requirements.txt
```

This will allow HTML generation. LaTeX requires additional tools to be installed. `sudo` is omitted since containers
will run as root by default. Windows user might see some permission issue and must use an elevated (run as Admin) terminal.

```console
$ apt-get -y install texlive-full

OR

$ apt-get -y install texlive-latex-recommended texlive-latex-extra texlive-fonts-recommended latexmk
```

There is also a script to copy the documentation files and folders directly onto the public release repository. Hence, you must
also clone the public release repo, but submodules init are not necessary.

```console
$ git clone git@github.com:robotic-vision-lab/UR-Robotiq-Integrated-Driver.git
```

## Building the Docs

Simply run `make html` to generate HTML or `make latexpdf` to generate LaTeX PDF documentation. The necessary files or folders
will be generated in `build/html` or `build/latex` respectively.

> :warning: The script requires that this repository and the UR-Robotiq-Integrated-Driver are in the same parent directory.

After generation, run the script using `sh update_documentation.sh` **FROM THE HOST** to move and overwrite files on the public release repository and primary folder of this repository.
