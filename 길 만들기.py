import pandas as pd
import requests
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

# -----------------------------
# 1. 주소 → 좌표 변환 함수
# -----------------------------
def address_to_coord(address, api_key):
    """
    카카오 REST API를 이용해 주소를 (위도, 경도)로 변환
    """
    url = "https://dapi.kakao.com/v2/local/search/address.json"
    headers = {"Authorization": f"KakaoAK {api_key}"}
    params = {"query": address}

    res = requests.get(url, headers=headers, params=params)
    if res.status_code != 200:
        raise Exception(f"주소 변환 실패: {res.text}")

    documents = res.json().get("documents")
    if not documents:
        raise Exception(f"주소를 찾을 수 없음: {address}")

    y = float(documents[0]["y"])  # 위도
    x = float(documents[0]["x"])  # 경도
    return (y, x)


# -----------------------------
# 2. CSV 불러오기 → 좌표 리스트 생성
# -----------------------------
def load_addresses(csv_file, api_key):
    """
    CSV 파일에서 주소를 읽고, 각 주소를 좌표로 변환
    CSV 형식: name,address
    """
    df = pd.read_csv(csv_file)
    coords = []
    for addr in df['address']:
        coords.append(address_to_coord(addr, api_key))
    return coords


# -----------------------------
# 3. 다음지도 API로 거리 행렬 만들기
# -----------------------------
def build_distance_matrix(locations, api_key):
    """
    locations 리스트를 받아서
    각 지점 사이의 거리(km)를 계산한 2차원 리스트(행렬) 반환
    """
    n = len(locations)
    matrix = [[0]*n for _ in range(n)]
    url = "https://apis-navi.kakaomobility.com/v1/distance"

    for i in range(n):
        for j in range(n):
            if i != j:
                params = {
                    "origin": f"{locations[i][1]},{locations[i][0]}",      # 경도,위도
                    "destination": f"{locations[j][1]},{locations[j][0]}", # 경도,위도
                }
                headers = {"Authorization": f"KakaoAK {api_key}"}

                res = requests.get(url, params=params, headers=headers)
                if res.status_code != 200:
                    raise Exception(f"거리 API 호출 실패: {res.text}")

                data = res.json()
                distance_m = data["distance"]  # m 단위
                matrix[i][j] = distance_m / 1000.0  # km 단위

    return matrix


# -----------------------------
# 4. OR-Tools로 TSP 최적화
# -----------------------------
def solve_tsp(distance_matrix):
    """
    거리 행렬을 입력받아 최적 경로와 총 거리를 반환
    """
    n = len(distance_matrix)
    manager = pywrapcp.RoutingIndexManager(n, 1, 0)  # 노드 수, 차량 수, 출발지
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(distance_matrix[from_node][to_node] * 1000)  # km → m

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC

    solution = routing.SolveWithParameters(search_parameters)

    if solution:
        index = routing.Start(0)
        route = []
        total_distance = 0
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            route.append(node)
            next_index = solution.Value(routing.NextVar(index))
            total_distance += routing.GetArcCostForVehicle(index, next_index, 0)
            index = next_index
        route.append(manager.IndexToNode(index))
        return route, total_distance / 1000
    else:
        return None, None


# -----------------------------
# 5. 메인 실행
# -----------------------------
def main():
    api_key = "여기에_카카오_REST_API_KEY_입력"  # 본인 REST API 키
    csv_file = "locations.csv"                    # 주소 CSV 파일

    # CSV → 좌표 변환
    locations = load_addresses(csv_file, api_key)

    # 거리 행렬 생성
    distance_matrix = build_distance_matrix(locations, api_key)

    # OR-Tools로 최적 경로 계산
    route, total_distance = solve_tsp(distance_matrix)

    # 결과 출력
    if route:
        print("최적 경로 (CSV 행 번호 기준):", route)
        print("총 거리:", round(total_distance, 2), "km")
    else:
        print("경로 계산 실패")


if __name__ == "__main__":
    main()
