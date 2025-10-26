import requests
from settings import API_IMG_IP, API_PORT

def test_algo_api():
    """Test if algorithm API server is accessible"""
    url = f"http://{API_IMG_IP}:{API_PORT}/status"
    try:
        response = requests.get(url, timeout=1)
        if response.status_code == 200:
            print("Algorithm API is up and running!")
            return True
        print(f"Algorithm API returned status code: {response.status_code}")
        return False
    except ConnectionError:
        print("Algorithm API Connection Error")
        return False
    except requests.Timeout:
        print("Algorithm API Timeout")
        return False
    except Exception as e:
        print(f"Algorithm API Exception: {e}")
        return False

if __name__ == "__main__":
    test_algo_api()
