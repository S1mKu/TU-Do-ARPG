workspace_path=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )/../../ros_ws

FOUND_PROBLEMS=false

# Check python code
PYTHON_DIFF=$(autopep8 --diff --recursive --aggressive --aggressive --exclude="$workspace_path/src/external_packages,$workspace_path/build,$workspace_path/devel,*.cfg" $workspace_path)

if echo $PYTHON_DIFF | grep -c +++ > /dev/null ; then
    echo "Found formatting problems in the python code."
    echo "Suggested changes:"
    echo "$PYTHON_DIFF"
    FOUND_PROBLEMS=true
fi


if $FOUND_PROBLEMS == true ; then
    echo "Cancelling the build due to badly formatted python code."
    echo "Please run ./scripts/format/format-src.sh"
    exit 1
else
    exit 0
fi
