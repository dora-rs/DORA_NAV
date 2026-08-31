"""
Printer Node — receives DORA outputs and prints them
Author: kaushikteja26
"""
import dora

def main():
    node = dora.Node()
    print("\n[Printer Node] Started — waiting for data...\n")
    for event in node:
        if event["type"] == "INPUT":
            print(f"  [Printer] Received: {event['id']}")
            print(f"  Data: {event['value']}\n")

if __name__ == "__main__":
    main()
