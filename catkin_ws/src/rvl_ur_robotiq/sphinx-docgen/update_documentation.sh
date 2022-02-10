#!/usr/bin/env sh

# script folder; https://stackoverflow.com/a/4774063
SCRIPT_DIR="$( cd -- "$(dirname "$0")" >/dev/null 2>&1 ; pwd -P )"

# build dir
SRC_HTML="$SCRIPT_DIR/build/html"
SRC_PDF="$SCRIPT_DIR/build/latex/rvl_driver_documentation.pdf"

# destinations
LOCAL_DOC="$( cd -- "$SCRIPT_DIR/../../../../documentation" >/dev/null 2>&1 ; pwd -P )"
DRVER_DOC="$( cd -- "$LOCAL_DOC/../../UR-Robotiq-Integrated-Driver/documentation" >/dev/null 2>&1 ; pwd -P )"

# echo $SCRIPT_DIR
echo $SRC_HTML
echo $SRC_PDF
echo $LOCAL_DOC
echo $DRVER_DOC

cp -r $SRC_HTML $LOCAL_DOC
cp -r $SRC_PDF $LOCAL_DOC

echo "Local documentation updated."

while true; do
    read -p "Do you want to update UR-Robotiq-Integrated-Driver documentation as well [YyNn]? " yn
    case $yn in
        [Yy]* ) 
            cp -r $SRC_HTML $DRVER_DOC;
            cp -r $SRC_PDF $DRVER_DOC;
            echo "UR-Integrated-Driver documentation updated."
            break;;
        [Nn]* ) exit;;
        * ) echo "Please answer yes or no.";;
    esac
done

# cp -rv $SRC_HTML $DRVER_DOC
# cp -rv $SRC_PDF $DRVER_DOC