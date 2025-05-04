# pip install pyserial - do basha jak nie ma biblioteki

import serial
import time

def main():
    # Ustawienia portu COM
    port_name = 'COM3'       # ← zmień na swój port (np. 'COM4', '/dev/ttyUSB0' dla Linux)
    baud_rate = 9600         # Szybkość transmisji
    timeout = 1              # Timeout na odbiór (sekundy)

    try:
        # Otwórz port
        ser = serial.Serial(port=port_name, baudrate=baud_rate, timeout=timeout)
        time.sleep(2)  # Poczekaj na inicjalizację (ważne dla Arduino)

        print(f"Połączono z {port_name}")

        while True:
            # Wysyłanie danych
            msg = input("Wyślij (lub 'exit'): ")
            if msg.lower() == 'exit':
                break

            ser.write(msg.encode('utf-8'))

            # Odbieranie odpowiedzi
            response = ser.readline().decode('utf-8').strip()
            if response:
                print(f"Odebrano: {response}")
            else:
                print("Brak odpowiedzi.")

        ser.close()
        print("Połączenie zakończone.")

    except serial.SerialException as e:
        print(f"Błąd połączenia: {e}")

if __name__ == "__main__":
    main()