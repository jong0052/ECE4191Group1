import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522

def readTag():
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
        print(f"{out}")