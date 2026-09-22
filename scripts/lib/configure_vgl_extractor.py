#!/usr/bin/env python3
"""Copy an exporter textproto with a fixed ALIKED NCHW input profile."""

import argparse
from pathlib import Path
import re


def block(text, name):
    # Mask comments and quoted strings while retaining offsets into the original.
    masked = re.sub(r'"(?:\\.|[^"\\])*"|#[^\n]*|//[^\n]*',
                    lambda match: ' ' * len(match.group()), text)
    matches = list(re.finditer(r'\b' + re.escape(name) + r'\s*\{', masked))
    if len(matches) != 1:
        raise ValueError(f"Expected exactly one {name} block, found {len(matches)}")
    start = matches[0].end()
    depth = 1
    for end in range(start, len(masked)):
        depth += (masked[end] == '{') - (masked[end] == '}')
        if depth == 0:
            return start, end
    raise ValueError(f"Unclosed {name} block")


def configure(text, width, height):
    if width <= 0 or height <= 0:
        raise ValueError("width and height must be positive")
    start, end = block(text, 'aliked_detector')
    detector = text[start:end]
    config_start, config_end = block(detector, 'tensorrt_config')
    config = detector[config_start:config_end]
    dims_start, dims_end = block(config, 'input_dimension')
    dims = config[dims_start:dims_end]
    for field in ('min', 'opt', 'max'):
        values = re.findall(r'\b' + field + r'\s*:\s*(\d+)', dims)
        if len(values) != 4 or values[:2] != ['1', '3']:
            raise ValueError(f"Expected NCHW ALIKED {field} shape starting with 1, 3")
    replacement = '\n' + ''.join(
        f'      {field}: {value}\n'
        for field in ('min', 'opt', 'max')
        for value in (1, 3, height, width)
    ) + '    '
    absolute_start = start + config_start + dims_start
    absolute_end = start + config_start + dims_end
    return text[:absolute_start] + replacement + text[absolute_end:]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('destination', type=Path)
    parser.add_argument('--width', type=int, required=True)
    parser.add_argument('--height', type=int, required=True)
    args = parser.parse_args()
    if args.source.resolve() == args.destination.resolve():
        parser.error('source and destination must be different')
    try:
        result = configure(args.source.read_text(), args.width, args.height)
    except ValueError as error:
        parser.error(str(error))
    args.destination.write_text(result)


if __name__ == '__main__':
    main()
