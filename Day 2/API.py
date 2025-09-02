import urequests

#Example from https://docs.micropython.org/en/latest/esp8266/tutorial/network_basics.html 
def wifi_connect():
    import network
    import secrets
    sta_if = network.WLAN(network.WLAN.IF_STA)
    if not sta_if.isconnected():
        print('connecting to network...')
        sta_if.active(True)
        sta_if.connect(secrets.SSID, secrets.PWD)
        while not sta_if.isconnected():
            pass
    print('network config:', sta_if.ipconfig('addr4'))
    
wifi_connect()


# Define the API endpoint
url  = "https://corporatebs-generator.sameerkumar.website/"

# Make a GET request
response = urequests.get(url)

# Check if the request was successful
if response.status_code == 200:
    # Parse the JSON response
    data = response.json()
    print("Response data:", data)
else:
    print(f"Error: {response.status_code}")
    print(response.text)
