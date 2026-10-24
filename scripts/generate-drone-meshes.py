#!/usr/bin/env python3
"""Generate the original, metre-scale landing-drone visual meshes (stdlib only).

The SDF's links, collisions, inertias, sensors and flight plugins are unchanged.
Every solid is triangulated with outward normals and checked for closed edges.
"""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
MODEL = ROOT / "ros/ocean_sim/models/landing_drone"
MESHES = MODEL / "meshes"


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def scale(a, s):
    return tuple(x * s for x in a)


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def unit(a):
    length = math.sqrt(dot(a, a))
    if length < 1e-12:
        raise ValueError("Degenerate mesh normal")
    return scale(a, 1 / length)


class Mesh:
    def __init__(self, name):
        self.name, self.vertices, self.faces, self.parts = name, [], [], []

    def solid(self, label, vertices, faces):
        """Append a closed solid, correcting its winding before export."""
        edges = Counter(tuple(sorted((face[i], face[(i + 1) % 3])))
                        for face in faces for i in range(3))
        if not edges or any(count != 2 for count in edges.values()):
            raise ValueError(f"{self.name}/{label}: non-manifold edges")
        volume = sum(dot(vertices[a], cross(vertices[b], vertices[c])) / 6
                     for a, b, c in faces)
        if abs(volume) < 1e-12:
            raise ValueError(f"{self.name}/{label}: zero volume")
        if volume < 0:
            faces = [(a, c, b) for a, b, c in faces]
            volume = -volume
        for a, b, c in faces:
            unit(cross(sub(vertices[b], vertices[a]), sub(vertices[c], vertices[a])))
        offset = len(self.vertices)
        self.vertices.extend(vertices)
        self.faces.extend(tuple(i + offset for i in face) for face in faces)
        self.parts.append({"name": label, "vertices": len(vertices),
                           "triangles": len(faces), "closed_edges": True,
                           "outward_winding": True, "volume_m3": volume})

    def loft(self, label, rings):
        count = len(rings[0])
        assert all(len(ring) == count for ring in rings)
        vertices = [v for ring in rings for v in ring]
        faces = []
        for ring in range(len(rings) - 1):
            for i in range(count):
                a, b = ring * count + i, ring * count + (i + 1) % count
                faces.extend([(a, b, b + count), (a, b + count, a + count)])
        for ring, reverse in ((0, True), (len(rings) - 1, False)):
            points = rings[ring]
            center = tuple(sum(v[axis] for v in points) / count for axis in range(3))
            ci = len(vertices)
            vertices.append(center)
            for i in range(count):
                a, b = ring * count + i, ring * count + (i + 1) % count
                faces.append((ci, b, a) if reverse else (ci, a, b))
        self.solid(label, vertices, faces)

    def cylinder(self, label, p0, p1, r0, r1=None, segments=24):
        direction = unit(sub(p1, p0))
        reference = (0, 0, 1) if abs(direction[2]) < .9 else (1, 0, 0)
        u = unit(cross(reference, direction))
        v = cross(direction, u)
        rings = []
        for center, radius in ((p0, r0), (p1, r0 if r1 is None else r1)):
            rings.append([add(center, add(scale(u, radius * math.cos(i * 2 * math.pi / segments)),
                                          scale(v, radius * math.sin(i * 2 * math.pi / segments))))
                          for i in range(segments)])
        self.loft(label, rings)

    def beam(self, label, start, end, width, height, end_width=None):
        direction = unit((end[0] - start[0], end[1] - start[1], 0))
        side = (-direction[1], direction[0], 0)
        rings = []
        for point, w in ((start, width), (end, end_width or width)):
            rings.append([add(point, add(scale(side, s * w / 2), (0, 0, z * height / 2)))
                          for s, z in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
        self.loft(label, rings)

    def sphere(self, label, center, radius, segments=24, rows=12):
        vertices = [add(center, (0, 0, radius))]
        for row in range(1, rows):
            angle = math.pi * row / rows
            vertices.extend(add(center, (radius * math.sin(angle) * math.cos(2 * math.pi * i / segments),
                                          radius * math.sin(angle) * math.sin(2 * math.pi * i / segments),
                                          radius * math.cos(angle))) for i in range(segments))
        bottom = len(vertices)
        vertices.append(add(center, (0, 0, -radius)))
        faces = [(0, 1 + i, 1 + (i + 1) % segments) for i in range(segments)]
        for row in range(rows - 2):
            for i in range(segments):
                a, b = 1 + row * segments + i, 1 + row * segments + (i + 1) % segments
                faces.extend([(a, a + segments, b + segments), (a, b + segments, b)])
        last = 1 + (rows - 2) * segments
        faces.extend((bottom, last + (i + 1) % segments, last + i) for i in range(segments))
        self.solid(label, vertices, faces)

    def write(self):
        # Per-face normals keep chamfers crisp; 24/32-sided circular pieces are
        # deliberately finely faceted. Explicit normals also work in Ogre2.
        positions = " ".join(f"{v:.9g}" for point in self.vertices for v in point)
        normals = [unit(cross(sub(self.vertices[b], self.vertices[a]),
                              sub(self.vertices[c], self.vertices[a]))) for a, b, c in self.faces]
        normal_text = " ".join(f"{v:.9g}" for normal in normals for v in normal)
        indices = " ".join(f"{index} {face_i}" for face_i, face in enumerate(self.faces) for index in face)
        content = f'''<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset><contributor><author>Ocean simulation maintainers</author>
    <authoring_tool>generate-drone-meshes.py; original GPL-3.0-or-later geometry</authoring_tool>
    </contributor><created>2026-10-24T00:00:00Z</created><modified>2026-10-24T00:00:00Z</modified>
    <unit name="meter" meter="1"/><up_axis>Z_UP</up_axis></asset>
  <library_geometries><geometry id="{self.name}" name="{self.name}"><mesh>
    <source id="positions"><float_array id="positions-array" count="{len(self.vertices) * 3}">{positions}</float_array>
      <technique_common><accessor source="#positions-array" count="{len(self.vertices)}" stride="3">
        <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
      </accessor></technique_common></source>
    <source id="normals"><float_array id="normals-array" count="{len(normals) * 3}">{normal_text}</float_array>
      <technique_common><accessor source="#normals-array" count="{len(normals)}" stride="3">
        <param name="X" type="float"/><param name="Y" type="float"/><param name="Z" type="float"/>
      </accessor></technique_common></source>
    <vertices id="vertices"><input semantic="POSITION" source="#positions"/></vertices>
    <triangles count="{len(self.faces)}"><input semantic="VERTEX" source="#vertices" offset="0"/>
      <input semantic="NORMAL" source="#normals" offset="1"/><p>{indices}</p></triangles>
  </mesh></geometry></library_geometries>
  <library_visual_scenes><visual_scene id="Scene"><node id="{self.name}-node">
    <instance_geometry url="#{self.name}"/></node></visual_scene></library_visual_scenes>
  <scene><instance_visual_scene url="#Scene"/></scene>
</COLLADA>
'''
        path = MESHES / f"{self.name}.dae"
        path.write_text(content, encoding="utf-8", newline="\n")
        return {"file": path.name, "sha256": hashlib.sha256(content.encode()).hexdigest(),
                "vertices": len(self.vertices), "triangles": len(self.faces),
                "bounds_m": [[min(v[i] for v in self.vertices) for i in range(3)],
                             [max(v[i] for v in self.vertices) for i in range(3)]],
                "solids": self.parts}


def octagon(x, y, chamfer, z):
    return [(a, b, z) for a, b in ((-x + chamfer, -y), (x - chamfer, -y),
            (x, -y + chamfer), (x, y - chamfer), (x - chamfer, y),
            (-x + chamfer, y), (-x, y - chamfer), (-x, -y + chamfer))]


def physics_fingerprint(root):
    def item(node):
        return [node.tag, sorted(node.attrib.items()), " ".join((node.text or "").split()),
                [item(child) for child in node if child.tag != "visual"]]
    return hashlib.sha256(json.dumps(item(root), separators=(",", ":")).encode()).hexdigest()


def visual(name, mesh, color, *, metal=.0, rough=.6, emissive=False):
    rgba = " ".join(str(v) for v in (*color, 1))
    ambient = " ".join(str(round(v * .8, 5)) for v in (*color,)) + " 1"
    return ET.fromstring(f'''<visual name="{name}"><geometry><mesh>
      <uri>model://landing_drone/meshes/{mesh}.dae</uri></mesh></geometry>
      <material><ambient>{ambient}</ambient><diffuse>{rgba}</diffuse>
      <specular>0.25 0.25 0.25 1</specular>
      {'<emissive>' + rgba + '</emissive>' if emissive else ''}
      <pbr><metal><metalness>{metal}</metalness><roughness>{rough}</roughness></metal></pbr>
      </material></visual>''')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="verify the saved physics fingerprint after regeneration")
    args = parser.parse_args()
    source = MODEL / "model.sdf"
    tree = ET.parse(source)
    root = tree.getroot()
    before = physics_fingerprint(root)
    previous = MESHES / "geometry_manifest.json"
    if previous.exists():
        assert json.loads(previous.read_text())["physics_sha256_before"] == before, "Non-visual model changed"
    MESHES.mkdir(parents=True, exist_ok=True)
    names = ("airframe_carbon", "canopy_orange", "trim_graphite", "fasteners_aluminum",
             "camera_housing", "camera_lens", "navigation_front", "navigation_rear", "foot_rubber",
             "motor_bell", "motor_copper", "rotor_blade_ccw", "rotor_blade_cw",
             "propeller_tip_ccw", "propeller_tip_cw")
    meshes = {name: Mesh(name) for name in names}
    carbon, orange, trim = (meshes[name] for name in names[:3])
    aluminum, camera, lens, front, rear, rubber = (meshes[name] for name in names[3:9])
    carbon.loft("lower structural shell", [octagon(.105, .070, .016, -.05),
                octagon(.130, .090, .022, -.027), octagon(.130, .090, .022, .012)])
    orange.loft("chamfered canopy", [octagon(.130, .090, .022, -.003),
                octagon(.127, .087, .024, .034), octagon(.105, .068, .025, .061)])
    trim.loft("top service hatch", [octagon(.061, .047, .012, .061), octagon(.059, .045, .012, .063)])
    # A connected carbon X frame, reinforced at both the shell and motor seats.
    anchors = []
    for sx, sy, label in ((1, 1, "fl"), (1, -1, "fr"), (-1, 1, "rl"), (-1, -1, "rr")):
        p = lambda x, y, z: (sx * x, sy * y, z)
        carbon.beam(f"{label} tapered arm", p(.066, .047, -.006), p(.230, .230, .006), .060, .034, .046)
        orange.beam(f"{label} arm saddle", p(.095, .078, .007), p(.135, .124, .010), .064, .015, .056)
        carbon.cylinder(f"{label} motor seat", p(.230, .230, -.012), p(.230, .230, .027), .032, .029, 32)
        aluminum.cylinder(f"{label} stator support", p(.230, .230, .022), p(.230, .230, .037), .026, .025, 32)
        # Landing struts start INSIDE the structural shell and end inside the
        # existing spherical collision feet, so no piece floats in space.
        anchor, knee, ankle, foot = p(.106, .069, -.025), p(.190, .180, -.166), p(.300, .300, -.410), p(.300, .300, -.425)
        carbon.cylinder(f"{label} upper landing strut", anchor, knee, .014, .012)
        carbon.cylinder(f"{label} lower landing strut", knee, ankle, .012, .011)
        carbon.sphere(f"{label} knee joint", knee, .014)
        carbon.cylinder(f"{label} motor-to-leg brace", p(.220, .220, -.005), p(.244, .239, -.286), .008)
        orange.cylinder(f"{label} upper attachment cuff", anchor, p(.123, .091, -.053), .019, .017)
        aluminum.cylinder(f"{label} ankle sleeve", p(.293, .292, -.393), foot, .015, .012)
        rubber.sphere(f"{label} contact foot", foot, .025, 32, 16)
        for bx, by in ((.100, .070), (.215, .230), (.245, .230)):
            aluminum.cylinder(f"{label} frame bolt {bx}", p(bx, by, .024 if bx < .15 else .027),
                              p(bx, by, .029 if bx < .15 else .031), .004, segments=12)
        anchors.append({"name": label, "shell_attachment_m": anchor, "knee_m": knee,
                        "foot_center_m": foot, "foot_radius_m": .025,
                        "motor_center_m": p(.23, .23, .045)})
    # Housing ends at z=-.099; the physical optical origin is z=-.100.
    # The thin lens surface is behind that origin, outside the forward frustum.
    camera.loft("camera mount", [octagon(.026, .026, .006, -.043), octagon(.031, .031, .007, -.067),
                octagon(.027, .027, .007, -.090)])
    camera.cylinder("lens barrel", (0, 0, -.088), (0, 0, -.099), .021, .019, 32)
    aluminum.cylinder("lens retaining ring", (0, 0, -.096), (0, 0, -.0992), .020, .020, 32)
    lens.cylinder("optical glass", (0, 0, -.0988), (0, 0, -.0995), .016, .016, 32)
    for sx in (-1, 1):
        target = front if sx > 0 else rear
        target.loft("front lamps" if sx > 0 else "rear lamps",
                    [[(sx * .130 + x, y, z) for x, y, z in octagon(.004, .039, .003, h)]
                     for h in (.023, .033)])
    bell, copper = meshes["motor_bell"], meshes["motor_copper"]
    bell.cylinder("rotating motor bell", (0, 0, -.011), (0, 0, .017), .025, .023, 32)
    bell.cylinder("motor top shoulder", (0, 0, .017), (0, 0, .022), .023, .015, 32)
    bell.cylinder("raised propeller shaft", (0, 0, .019), (0, 0, .049), .006, .005, 24)
    bell.cylinder("propeller hub", (0, 0, .038), (0, 0, .048), .014, .012, 24)
    copper.cylinder("copper bell stripe", (0, 0, -.004), (0, 0, .000), .0254, .0254, 32)
    # Two solid, swept tapered blades; opposing pitch directions match CW/CCW.
    for direction, name in ((1, "rotor_blade_ccw"), (-1, "rotor_blade_cw")):
        blade = meshes[name]
        tips = meshes["propeller_tip_ccw" if direction > 0 else "propeller_tip_cw"]
        for side in (1, -1):
            rings = []
            for radius, chord, sweep, pitch in ((.010, .017, 0, .0025), (.038, .027, .003, .004),
                                                (.092, .024, .010, .003), (.130, .012, .015, .001)):
                ring = []
                for ysign, zsign in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
                    x, y = radius, direction * sweep + ysign * chord / 2
                    z = .043 + ysign * direction * pitch + zsign * .00125
                    ring.append((side * x, side * y, z))
                rings.append(ring)
            blade.loft(f"{'CCW' if direction > 0 else 'CW'} blade {side}", rings)
            # A 0.5 mm solid stripe overlaps the last 14 mm of blade surface;
            # its lower 0.25 mm is inside the blade, so it cannot float above it.
            tip_rings = []
            for radius in (.116, .130):
                t = (radius - .092) / (.130 - .092)
                chord, sweep, pitch = .024 - .012 * t, .010 + .005 * t, .003 - .002 * t
                tip_rings.append([(side * radius, side * (direction * sweep + ys * chord / 2),
                                   .043 + ys * direction * pitch + dz)
                                  for ys, dz in ((-1, .0010), (1, .0010), (1, .0015), (-1, .0015))])
            tips.loft(f"attached blade tip marker {side}", tip_rings)
    reports = [mesh.write() for mesh in meshes.values()]
    model = root.find("model")
    for link in model.findall("link"):
        for item in list(link.findall("visual")):
            link.remove(item)
    base = model.find("link[@name='base_link']")
    visual_specs = [("connected_carbon_frame", "airframe_carbon", (.075, .095, .115), .25, .55),
                    ("orange_canopy", "canopy_orange", (.96, .245, .045), .05, .38),
                    ("service_panel", "trim_graphite", (.105, .125, .145), .25, .48),
                    ("hardware_and_motor_supports", "fasteners_aluminum", (.58, .64, .69), .75, .30),
                    ("downward_camera_housing", "camera_housing", (.055, .065, .080), .12, .48),
                    ("downward_camera_glass", "camera_lens", (.055, .22, .29), .55, .13),
                    ("rubber_contact_feet", "foot_rubber", (.035, .045, .055), .0, .9)]
    for name, mesh, color, metal, rough in visual_specs:
        base.append(visual(name, mesh, color, metal=metal, rough=rough))
    base.append(visual("front_navigation_light", "navigation_front", (.05, .90, .32), rough=.3, emissive=True))
    base.append(visual("rear_navigation_light", "navigation_rear", (.95, .08, .025), rough=.3, emissive=True))
    for i in range(4):
        rotor = model.find(f"link[@name='rotor_{i}']")
        rotor.append(visual("motor_bell_and_shaft", "motor_bell", (.23, .27, .30), metal=.75, rough=.3))
        rotor.append(visual("motor_copper_band", "motor_copper", (.68, .27, .08), metal=.75, rough=.3))
        rotor.append(visual("shaped_propeller", "rotor_blade_ccw" if i < 2 else "rotor_blade_cw",
                            (.075, .095, .115), metal=.15, rough=.48))
        rotor.append(visual("attached_propeller_tip_marks", "propeller_tip_ccw" if i < 2 else "propeller_tip_cw",
                            (.98, .32, .065), metal=.05, rough=.45))
    after = physics_fingerprint(root)
    assert after == before, "A non-visual model element changed"
    ET.indent(tree, space="  ")
    tree.write(source, encoding="utf-8", xml_declaration=True)
    manifest = {"generator": "scripts/generate-drone-meshes.py", "units": "metres", "up_axis": "Z",
                "license": "GPL-3.0-or-later; original project geometry",
                "physics_sha256_before": before, "physics_sha256_after": after,
                "nonvisual_xml_unchanged": before == after, "attachments": anchors,
                "camera_optical_origin_m": [0, 0, -.1], "lowest_visual_point_m": -.45,
                "propeller_plane_body_z_m": .088, "motor_link_body_z_m": .045,
                "mesh_count": len(reports), "meshes": reports}
    previous.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps({"meshes": len(reports), "triangles": sum(m["triangles"] for m in reports),
                      "closed_solids": sum(len(m["solids"]) for m in reports),
                      "physics_unchanged": before == after, "physics_sha256": after}, indent=2))


if __name__ == "__main__":
    main()
