# Script to run Python with the 'projectcourse' conda environment.
# Behavior:
# 1) If the explicit python.exe exists under the user's Anaconda envs path, run it.
# 2) Else if `conda` is available on PATH, use `conda run -n projectcourse python`.
# 3) Otherwise print a helpful error and exit with non-zero status.

$pyPath = "C:\Users\31627\anaconda3\envs\projectcourse\python.exe"
if (Test-Path $pyPath) {
	& $pyPath @args
	exit $LASTEXITCODE
}

if (Get-Command conda -ErrorAction SilentlyContinue) {
	& conda run -n projectcourse python @args
	exit $LASTEXITCODE
}

Write-Error "Could not find Python for conda env 'projectcourse'. Checked: $pyPath and 'conda' command availability.\nTo create the env (example): conda create -n projectcourse python=3.11 -y"
exit 1