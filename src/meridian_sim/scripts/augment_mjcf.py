#!/usr/bin/env python3
"""
Augment the base ur5e.xml with:
  - ft_sensor_site on wrist_3_link
  - force + torque sensors
  - fingertip body on wrist_3_link
  - table in worldbody
  - USB-C receptacle target on table
"""
import os
import xml.etree.ElementTree as ET

ASSETS_DIR = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "meridian_description", "assets")
)
MJCF_PATH = os.path.join(ASSETS_DIR, "ur5e.xml")


def indent(elem, level=0):
    """Add pretty-print indentation in-place."""
    pad = "\n" + "  " * level
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = pad + "  "
        if not elem.tail or not elem.tail.strip():
            elem.tail = pad
        for child in elem:
            indent(child, level + 1)
        if not child.tail or not child.tail.strip():
            child.tail = pad
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = pad
    if not level:
        elem.tail = "\n"


def find_body(root, name):
    for body in root.iter("body"):
        if body.get("name") == name:
            return body
    return None


def main():
    ET.register_namespace("", "")
    tree = ET.parse(MJCF_PATH)
    root = tree.getroot()

    # ── 1. Add ft_sensor_site + fingertip to wrist_3_link ──────────────────
    wrist3 = find_body(root, "wrist_3_link")
    if wrist3 is None:
        # MuJoCo may rename bodies; try a suffix search
        for body in root.iter("body"):
            if "wrist_3" in (body.get("name") or ""):
                wrist3 = body
                break

    if wrist3 is None:
        raise RuntimeError("Could not find wrist_3_link body in MJCF")

    # Remove existing children with these names to allow re-runs
    for child in list(wrist3):
        if child.get("name") in ("ft_sensor_site", "fingertip"):
            wrist3.remove(child)

    site = ET.SubElement(wrist3, "site")
    site.set("name", "ft_sensor_site")
    site.set("pos", "0 0 0.0823")
    site.set("size", "0.01")
    site.set("type", "sphere")
    site.set("rgba", "1 0 0 0.5")

    fingertip_body = ET.SubElement(wrist3, "body")
    fingertip_body.set("name", "fingertip")
    fingertip_body.set("pos", "0 0 0.0823")
    fingertip_geom = ET.SubElement(fingertip_body, "geom")
    fingertip_geom.set("name", "fingertip_geom")
    fingertip_geom.set("type", "cylinder")
    fingertip_geom.set("size", "0.01 0.01")  # radius, half-length
    fingertip_geom.set("rgba", "0.8 0.8 0.8 1")

    # ── 2. Add table + USB-C target to worldbody ───────────────────────────
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("No worldbody element found")

    # Remove stale copies on re-run
    for child in list(worldbody):
        if child.get("name") in ("table", "usbc_target"):
            worldbody.remove(child)

    table = ET.SubElement(worldbody, "body")
    table.set("name", "table")
    table.set("pos", "0 0 0")
    table_geom = ET.SubElement(table, "geom")
    table_geom.set("name", "table_surface")
    table_geom.set("type", "box")
    table_geom.set("size", "0.5 0.5 0.02")  # half-extents
    table_geom.set("pos", "0 0 -0.02")       # top face at z=0
    table_geom.set("rgba", "0.7 0.6 0.5 1")
    table_geom.set("contype", "1")
    table_geom.set("conaffinity", "1")

    # USB-C target: reachable UR5e position ~0.4 m forward, on table top
    target = ET.SubElement(worldbody, "body")
    target.set("name", "usbc_target")
    target.set("pos", "0.4 0.0 0.0")
    target_geom = ET.SubElement(target, "geom")
    target_geom.set("name", "usbc_geom")
    target_geom.set("type", "box")
    target_geom.set("size", "0.0075 0.0035 0.005")  # half-extents of 15x7x10mm box
    target_geom.set("rgba", "0.1 0.1 0.8 1")
    target_site = ET.SubElement(target, "site")
    target_site.set("name", "target_site")
    target_site.set("pos", "0 0 0")
    target_site.set("size", "0.005")
    target_site.set("rgba", "0 1 0 0.8")

    # ── 3. Add sensor block ────────────────────────────────────────────────
    sensor_el = root.find("sensor")
    if sensor_el is None:
        sensor_el = ET.SubElement(root, "sensor")

    # Remove stale entries
    for child in list(sensor_el):
        if child.get("name") in ("ft_force", "ft_torque"):
            sensor_el.remove(child)

    force_s = ET.SubElement(sensor_el, "force")
    force_s.set("name", "ft_force")
    force_s.set("site", "ft_sensor_site")

    torque_s = ET.SubElement(sensor_el, "torque")
    torque_s.set("name", "ft_torque")
    torque_s.set("site", "ft_sensor_site")

    # ── 4. Save ────────────────────────────────────────────────────────────
    indent(root)
    tree.write(MJCF_PATH, encoding="unicode", xml_declaration=False)
    print(f"Augmented MJCF written to: {MJCF_PATH}")
    print("Added: ft_sensor_site, fingertip, table, usbc_target, force/torque sensors")


if __name__ == "__main__":
    main()
