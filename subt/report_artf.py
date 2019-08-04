"""
   Report artifact for STIX in Colorado April 2019
"""
import requests


#URL_BASE = "http://localhost:8000"  # demo
URL_BASE = "http://10.100.2.200:8000"  # Miami Tunnel
#URL_BASE = "http://10.100.1.200:8000"  # Army Tunnel

ARTF_TYPES = ['Survivor', 'Backpack', 'Cell Phone', 'Drill', 'Fire Extinguisher']

json_headers = {
#    "Authorization" : "Bearer subttesttoken123",  # demo
    "Authorization" : "Bearer NfEwAHEYsKqQkxSf",
    "Content-Type" : "application/json",
}


def get_status():
    print('Get Status')
    url = URL_BASE + "/api/status/"

    # Correct GET /api/status/ request
    response = requests.get(url, headers=json_headers)
    assert response.status_code == 200
    print(response.content)


def report_artf(artf_type, x, y, z):
    artifact_report_data = {
        "x": x,
        "y": y,
        "z": z,
        "type": artf_type,
    }
    print('Report', artifact_report_data)
    url = URL_BASE + "/api/artifact_reports/"

    # Correct POST /api/artifact_reports/ request
    response = requests.post(url, json=artifact_report_data, headers=json_headers)
    assert response.status_code == 201
    print(response.content)


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Report artifact to server')
    parser.add_argument('artf_type', help='Type of artifact', choices=ARTF_TYPES)
    parser.add_argument('x', help='X coordinate in meters', type=float)
    parser.add_argument('y', help='Y coordinate in meters', type=float)
    parser.add_argument('z', help='Z coordinate in meters', type=float)
    args = parser.parse_args()

    get_status()
    report_artf(args.artf_type, args.x, args.y, args.z)
    get_status()

# vim: expandtab sw=4 ts=4

