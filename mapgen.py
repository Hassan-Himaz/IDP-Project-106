data = {}

while True:
    a = input("Enter a key (two 2-letter words separated by space, or press Enter to stop): ").strip().lower()
    if not a:
        break

    parts = a.split()
    if len(parts) != 2 or any(len(part) != 2 for part in parts):
        print("Key must be exactly two 2-letter words. Try again.")
        continue

    values = []
    while True:
        b = input("Enter a value for the list (or press Enter to stop): ").strip().upper()
        if not b:
            break
        values.append(b)

    data[tuple(parts)] = values  # Store 'a' as a tuple and 'b' as a list

print(data)
#{('st', 'da'): ['S', 'R', 'L', 'LOAD'], ('da', 'ha'): ['L', 'S', 'R', 'UNLOAD'] ,('ha', 'da'): ['L', 'S', 'L', 'LOAD'], ('da', 'hb'): ['S', 'L', 'L', 'UNLOAD'], ('hb', 'da'): ['R', 'R', 'S', 'LOAD'], ('da', 'hc'): ['S', 'L', 'S', 'R', 'L', 'UNLOAD'], ('hc', 'da'): ['R', 'L', 'S', 'L', 'S', 'LOAD'], ('da', 'hd'): ['S', 'S', 'L', 'L', 'UNLOAD'], ('hd', 'da'): ['R', 'R', 'S', 'S', 'LOAD']}