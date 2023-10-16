import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522
import time

def readTag() -> str:
    reader = SimpleMFRC522()

    try:
        id, text = reader.read()
        print(text)
        
    finally:
        GPIO.cleanup()
    
    return text

if __name__ == "__main__":
    while (1):
        out = readTag()

         # Cry, 1 2 3 and 0 1 2.
        if (out.startswith("1")):
            print("Package Found (1 -> 0) (Left). Running in 3 Seconds.")
            time.sleep(3)
        elif (out.startswith("2")):
            print("Package Found (2 -> 1) (Middle). Running in 3 Seconds.")
            time.sleep(3)
        elif (out.startswith("3")):
            print("Package Found (3 -> 2) (Right). Running in 3 Seconds.")
            time.sleep(3)
        print(f"{out}")