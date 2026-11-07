#!/usr/bin/env python3
"""Generate VS Code C/C++ settings from the installed Arduino board package."""

import argparse
import json
from pathlib import Path
import shlex
import subprocess


ROOT = Path(__file__).resolve().parents[1]
BUILD = ROOT / ".vscode" / "intellisense"


def expand_response_files(arguments):
    expanded = []
    for argument in arguments:
        if argument.startswith("@"):
            expanded.extend(expand_response_files(shlex.split(Path(argument[1:]).read_text())))
        else:
            expanded.append(argument)
    return expanded


def vscode_arguments(arguments):
    """Expose GCC response-file and prefixed includes to C/C++ IntelliSense."""
    normalized = []
    prefix = ""
    iterator = iter(expand_response_files(arguments))
    for argument in iterator:
        if argument == "-iprefix":
            prefix = next(iterator)
        elif argument == "-iwithprefixbefore":
            normalized.append("-I" + prefix + next(iterator))
        elif argument == "-iwithprefix":
            normalized.extend(["-idirafter", prefix + next(iterator)])
        else:
            normalized.append(argument)
    return normalized


def include_paths(arguments):
    paths = []
    prefix = ""
    iterator = iter(arguments)
    for argument in iterator:
        if argument == "-iprefix":
            prefix = next(iterator)
        elif argument == "-iwithprefixbefore":
            paths.append(str(Path(prefix) / next(iterator)))
        elif argument == "-I":
            paths.append(next(iterator))
        elif argument.startswith("-I"):
            paths.append(argument[2:])
    return list(dict.fromkeys(paths))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fqbn", default="esp32:esp32:esp32s3")
    args = parser.parse_args()
    BUILD.mkdir(parents=True, exist_ok=True)
    subprocess.run(
        ["arduino-cli", "compile", "--fqbn", args.fqbn,
         "--build-path", str(BUILD), "--only-compilation-database", str(ROOT)],
        check=True,
    )

    database_path = BUILD / "compile_commands.json"
    database = json.loads(database_path.read_text())
    sketch_copy = BUILD / "sketch"
    sketch_entries = []
    for entry in database:
        entry["arguments"] = vscode_arguments(entry["arguments"])
        source = Path(entry["file"])
        if not source.is_relative_to(sketch_copy):
            continue
        relative = source.relative_to(sketch_copy)
        if relative.name.endswith(".ino.cpp"):
            relative = relative.with_name(relative.name[:-4])
        original = ROOT / relative
        if not original.is_file():
            continue
        entry["arguments"] = [str(original) if item == str(source) else item
                              for item in entry["arguments"]]
        entry["file"] = str(original)
        sketch_entries.append(entry)
    if not sketch_entries:
        raise RuntimeError("Arduino CLI produced no sketch compilation entries")
    database_path.write_text(json.dumps(database, indent=2) + "\n")

    # Header and sketch fallback configuration for files without an exact database entry.
    arguments = sketch_entries[0]["arguments"]
    defines = list(dict.fromkeys(arg[2:] for arg in arguments if arg.startswith("-D")))
    configuration = {
        "version": 4,
        "configurations": [{
            "name": "Arduino ESP32-S3",
            "compileCommands": str(database_path),
            "compilerPath": sketch_entries[0]["arguments"][0],
            "includePath": include_paths(arguments),
            "defines": defines,
        }],
    }
    (ROOT / ".vscode" / "c_cpp_properties.json").write_text(
        json.dumps(configuration, indent=2) + "\n"
    )
    print(f"Configured {len(sketch_entries)} sketch files for VS Code ({args.fqbn}).")


if __name__ == "__main__":
    main()
