translation_value = -0.4
item_number = 1

while translation_value <= 0.6:
    print(f"Solid {{")
    print(f"  translation {translation_value} -0.1 0")
    print(f"  children [")
    print(f"    WoodenBox {{")
    print(f"      translation 0 0 0.055")
    print(f"      size 0.1 0.1 0.07")
    print(f"    }}")
    print(f"    WoodenPallet {{")
    print(f"      size 0.1 0.1 0.02")
    print(f"    }}")
    print(f"  ]")
    print(f"  name \"item {item_number}\"")
    print(f"}}
")
    
    translation_value += 0.1
    item_number += 1
