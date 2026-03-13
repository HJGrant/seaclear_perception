if not contains "/workspaces/oakd_isaac_ros/.tools/uv" $PATH
    # Prepending path in case a system-installed binary needs to be overridden
    set -x PATH "/workspaces/oakd_isaac_ros/.tools/uv" $PATH
end
