
def hex_to_rgb565(hex_str):
    hex_str = hex_str.lstrip('#')
    r = int(hex_str[0:2], 16)
    g = int(hex_str[2:4], 16)
    b = int(hex_str[4:6], 16)
    
    r5 = (r * 31) // 255
    g6 = (g * 63) // 255
    b5 = (b * 31) // 255
    
    rgb565 = (r5 << 11) | (g6 << 5) | b5
    return f"0x{rgb565:04X}"

colors = {
    "Background": "#0d1117",
    "Water": "#1a2b3c",
    "Park_Grass": "#1a2e1f",
    "Wood": "#162618",
    "Sand": "#CDAA7F",
    "Road_Trunk": "#e8a838",
    "Road_Primary": "#888888",
    "Road_Sec_Tert": "#d4d4d4",
    "Road_Minor": "#4a4a4a",
    "Route_Active": "#1A73E8",
    "Route_Alt": "#64B5F6",
    "Landuse_Residential": "#1c1f26",
    "Landuse_Commercial": "#1e2129",
    "Landuse_Industrial": "#1a1c22"
}

print("Colors header content:")
for name, hex_val in colors.items():
    print(f"// {name}: {hex_val} -> {hex_to_rgb565(hex_val)}")
