param(
    [Parameter(Mandatory=$true)]
    [string]$Target
)

# Define the symlink name
$SymlinkName = "main.cpp"

# Resolve the target file
$TargetFile = "$Target.cpp"

# Check if the target file exists
if (-Not (Test-Path ./src_all/$TargetFile)) {
    Write-Host "Error: Target file '$TargetFile' does not exist." -ForegroundColor Red
    exit 1
}

# Check if the symlink already exists
if (Test-Path ./src/$SymlinkName) {
    # Remove the existing symlink or file
    Remove-Item ./src/$SymlinkName -Force
    Write-Host "Removed existing symlink or file: $SymlinkName"
}

# Create the new symlink
New-Item -ItemType SymbolicLink -Path $PSScriptRoot/src/$SymlinkName -Target $PSScriptRoot/src_all/$TargetFile
Write-Host "Created symlink: $SymlinkName -> $TargetFile"
