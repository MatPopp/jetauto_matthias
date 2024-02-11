import requests 
    
def get_public_ip(): 
    try: 
        response = requests.get('https://httpbin.org/ip') 
        if response.status_code == 200: 
            ip_data = response.json() 
            public_ip = ip_data.get('origin') 
            return public_ip 
        else: 
            print(f"Failed to retrieve IP (Status code: {response.status_code})") 
    except Exception as e: 
        print(f"Error: {e}") 
    
# Get and print the public IP address 
public_ip = get_public_ip() 
print(f"Your public IP address is: {public_ip}") 

while True:
    pass