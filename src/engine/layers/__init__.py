"""Map data layer system — import/export/stream geospatial formats.

Supports KML 2.3, GeoJSON (RFC 7946), GPX 1.1, CoT XML, and CSV.
All parsers use only Python stdlib (xml.etree.ElementTree, json, csv).
"""

from engine.layers.layer import Layer, LayerFeature
from engine.layers.manager import LayerManager

__all__ = ["Layer", "LayerFeature", "LayerManager"]
