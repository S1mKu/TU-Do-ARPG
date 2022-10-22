workspace_path=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )/../../ros_ws

FOUND_PROBLEMS=false

# Check C++ code
for FILE in $(find "$workspace_path/src/" -path '*ros_ws/src/external_packages' -prune -o \( -name '*.h' -or -name '*.cpp' \) -print)
do
    if clang-format -i -style=file -output-replacements-xml $FILE | grep -c "<replacement " > /dev/null ; then
        echo "Formatting problem in:" $FILE
        FOUND_PROBLEMS=true
    fi
done

if $FOUND_PROBLEMS == true ; then
    echo "Cancelling the build due to badly formatted C++ code."
    echo "Please run ./scripts/format/format-src.sh"
    exit 1
else
    exit 0
fi
