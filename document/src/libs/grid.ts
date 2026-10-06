// 웹 패널이 쓰는 occupancy grid 모델 — python/slam/maps/occupancy_grid.py의 미러.
// 저장소 컨벤션과 동일하게 셀 인덱스는 (row, col) 이며 row 0이 이미지 최상단이다.
// world 좌표 변환도 여기 있다 — 좌표 프레임은 맵 레이어만 소유한다. 수출 JSON은 이미
// free_thresh를 적용한 '#'/'.' 행이라 파싱은 문자 하나면 끝난다.
import {Cell, Point} from "./trace/types";

export interface GridMap {
    name: string;
    width: number;    // cols
    height: number;   // rows
    // meters per cell — world 좌표 렌더링과 레이캐스트가 쓴다.
    resolution: number;
    // bottom-left 픽셀의 world 좌표 [x, y].
    origin: [number, number];
    // row-major 점유 여부. index = row * width + col.
    occupied: boolean[];
}

// tools/web_export가 만드는 맵 JSON ('#' = occupied).
export interface GridMapJson {
    name: string;
    width: number;
    height: number;
    resolution: number;
    free_thresh?: number;
    origin: [number, number];
    rows: string[];
}

export function parseGridMap(json: GridMapJson): GridMap {
    const occupied: boolean[] = new Array(json.width * json.height).fill(false)
    json.rows.forEach((row, r) => {
        for (let c = 0; c < json.width; c++) occupied[r * json.width + c] = row[c] === "#"
    })
    return {
        name: json.name, width: json.width, height: json.height,
        resolution: json.resolution, origin: json.origin, occupied,
    }
}

// python OccupancyGrid2D.cell_to_world의 미러 — 같은 산술 순서 (row 0 = 최상단 뒤집기 포함).
export const cellToWorld = (map: GridMap, c: Cell): Point => [
    map.origin[0] + (c[1] + 0.5) * map.resolution,
    map.origin[1] + ((map.height - 1 - c[0]) + 0.5) * map.resolution,
]

// python OccupancyGrid2D.world_to_cell의 미러.
export const worldToCell = (map: GridMap, p: Point): Cell => {
    const col = Math.floor((p[0] - map.origin[0]) / map.resolution)
    const row = (map.height - 1) - Math.floor((p[1] - map.origin[1]) / map.resolution)
    return [row, col]
}

// world 점 → 셀 좌표의 소수판 [row_f, col_f] — 렌더러의 disp()가 이것을 픽셀로 스케일한다.
// cell(정수) 중심은 정확히 [r+0.5, c+0.5]에 착지하므로 두 좌표계가 같은 픽셀을 가리킨다.
export const worldToCellFloat = (map: GridMap, p: Point): [number, number] =>
    [map.height - (p[1] - map.origin[1]) / map.resolution,
     (p[0] - map.origin[0]) / map.resolution]

export const inBounds = (map: GridMap, row: number, col: number): boolean =>
    row >= 0 && row < map.height && col >= 0 && col < map.width

// 경계 밖은 occupied가 아니다 (레이가 맵을 벗어나면 miss — 시나리오는 로봇이 자유 공간을
// 벗어나지 않음을 보장한다).
export const occupiedAt = (map: GridMap, row: number, col: number): boolean =>
    inBounds(map, row, col) && map.occupied[row * map.width + col]
