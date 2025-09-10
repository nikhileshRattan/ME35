SSID="tufts_eecs"
PWD="foundedin1883"
at_api_token ="patcxZbmZNg1bN4Vs.a778f0145d065ec3a566618df815be86798471426e21d75f06d67e247f84cf58"



import urequests
url = "https://worldtimeapi.org/api/timezone/America/New_York"
reply = urequests.get(url)
print(reply.json()['datetime'])