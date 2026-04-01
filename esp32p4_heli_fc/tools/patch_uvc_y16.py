#!/usr/bin/env python3
"""
Patch espressif/usb_host_uvc managed component to add Y16 (16-bit grayscale)
format support for FLIR Lepton / PureThermal thermal cameras.

Run automatically from CMakeLists.txt before build.
Safe to re-run: checks if patch is already applied.
"""

import os
import sys
import re

PROJ_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MANAGED = os.path.join(PROJ_DIR, "managed_components", "espressif__usb_host_uvc")

HEADER = os.path.join(MANAGED, "include", "usb", "uvc_host.h")
PARSING = os.path.join(MANAGED, "uvc_descriptor_parsing.c")

MARKER = "UVC_VS_FORMAT_Y16"


def patch_header(path):
    """Add UVC_VS_FORMAT_Y16 to the format enum."""
    with open(path, "r") as f:
        content = f.read()

    if MARKER in content:
        print(f"  [Y16] Header already patched: {path}")
        return False

    # Insert UVC_VS_FORMAT_Y16 after UVC_VS_FORMAT_H265
    new_content = content.replace(
        "UVC_VS_FORMAT_H265,",
        "UVC_VS_FORMAT_H265,\n"
        "    UVC_VS_FORMAT_Y16,        /*!< Y16 (16-bit grayscale) stream format. */",
    )

    if new_content == content:
        print(f"  [Y16] WARNING: Could not find insertion point in {path}", file=sys.stderr)
        return False

    with open(path, "w") as f:
        f.write(new_content)
    print(f"  [Y16] Patched header: {path}")
    return True


def patch_parsing(path):
    """Add Y16 GUID matching in uvc_desc_parse_format()."""
    with open(path, "r") as f:
        content = f.read()

    if MARKER in content:
        print(f"  [Y16] Parsing already patched: {path}")
        return False

    # After the YUY2 check in FORMAT_UNCOMPRESSED case, add Y16 check
    # Pattern: if (strncmp(guid, "YUY2", 4) == 0) { ret = UVC_VS_FORMAT_YUY2; }
    old_pattern = 'if (strncmp(guid, "YUY2", 4) == 0) {\n            ret = UVC_VS_FORMAT_YUY2;\n        }'
    new_code = (
        'if (strncmp(guid, "YUY2", 4) == 0) {\n'
        '            ret = UVC_VS_FORMAT_YUY2;\n'
        '        } else if (strncmp(guid, "Y16 ", 4) == 0 || strncmp(guid, "Y16\\x00", 4) == 0) {\n'
        '            ret = UVC_VS_FORMAT_Y16;\n'
        '        }'
    )

    new_content = content.replace(old_pattern, new_code)

    if new_content == content:
        # Try alternative whitespace patterns
        # Use regex for more flexible matching
        pattern = r'if\s*\(strncmp\(guid,\s*"YUY2",\s*4\)\s*==\s*0\)\s*\{\s*\n\s*ret\s*=\s*UVC_VS_FORMAT_YUY2;\s*\n\s*\}'
        match = re.search(pattern, content)
        if match:
            old_text = match.group(0)
            indent = "        "
            new_text = (
                old_text + f'\n{indent}}} else if (strncmp(guid, "Y16 ", 4) == 0 || strncmp(guid, "Y16\\x00", 4) == 0) {{\n'
                f'{indent}    ret = UVC_VS_FORMAT_Y16;\n'
            )
            # Remove trailing } from old_text replacement since we're adding else-if
            # Actually, the old_text already ends with }, we need to restructure
            # Let's do a simpler replacement: just add after the closing brace
            new_content = content.replace(
                old_text,
                old_text.rstrip('}') +
                '} else if (strncmp(guid, "Y16 ", 4) == 0 || strncmp(guid, "Y16\\x00", 4) == 0) {\n'
                '            ret = UVC_VS_FORMAT_Y16;\n'
                '        }'
            )

        if new_content == content:
            print(f"  [Y16] WARNING: Could not find YUY2 pattern in {path}", file=sys.stderr)
            print(f"  [Y16] Manual patch may be needed", file=sys.stderr)
            return False

    with open(path, "w") as f:
        f.write(new_content)
    print(f"  [Y16] Patched parsing: {path}")
    return True


def main():
    if not os.path.isdir(MANAGED):
        print(f"  [Y16] Component not yet downloaded: {MANAGED}")
        print(f"  [Y16] Run 'idf.py build' first, then re-run this script")
        return 0

    if not os.path.isfile(HEADER):
        print(f"  [Y16] Header not found: {HEADER}", file=sys.stderr)
        return 1

    if not os.path.isfile(PARSING):
        print(f"  [Y16] Parsing source not found: {PARSING}", file=sys.stderr)
        return 1

    h = patch_header(HEADER)
    p = patch_parsing(PARSING)

    if h or p:
        print("  [Y16] UVC Y16 format patch applied successfully")
    else:
        print("  [Y16] UVC Y16 format patch already applied (no changes)")

    return 0


if __name__ == "__main__":
    sys.exit(main())
