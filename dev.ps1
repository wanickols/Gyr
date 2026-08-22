param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$Command
)

if (-not $Command -or $Command.Count -eq 0) {
    $Command = @("cargo", "run")
}

podman run --rm `
    --mount type=bind,source="${PWD}",target=/workspace `
    --mount type=volume,source=rust-cargo-cache,target=/usr/local/cargo/registry `
    --mount type=volume,source=gyr-target,target=/workspace/target `
    -w /workspace `
    localhost/gyr-dev:latest `
    @Command