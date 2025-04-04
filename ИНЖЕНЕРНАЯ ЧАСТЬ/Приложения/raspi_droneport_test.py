# приложение для тестового запуска открытия-закрытия дронопорта и проверки его работы
import socket

def send_message_to_esp32(message, esp32_ip, esp32_port):
    try:
        
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        sock.settimeout(5)
        
        
        sock.connect((esp32_ip, esp32_port))
        
        
        sock.sendall(message.encode())
        
        print(f"message sent: {message}")
        
        
        response = sock.recv(1024)
        if response:
            print(f"response ESP32: {response.decode()}")
            
    except Exception as e:
        print(f"Fail: {e}")
    finally:
        sock.close()


if __name__ == "__main__":
    ESP32_IP = "192.168.50.211"  
    ESP32_PORT = 33300           
    
    message = input()
    send_message_to_esp32(message, ESP32_IP, ESP32_PORT)
