#!/usr/bin/env python3
# Copyright 2026 Adaptive Pivot-G2 Research Team
# Licensed under the Apache License, Version 2.0

"""Generate reproducible figures for the multi-map planner pilot."""

import csv
import math
from pathlib import Path
import statistics

from PIL import Image, ImageDraw, ImageFont
import yaml


ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / 'docs' / 'planner_map_assets'
FONT_REGULAR = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
FONT_BOLD = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
ENVIRONMENTS = ('open_arena', 'narrow_aisles', 'office_maze')
PLANNERS = (
    'NavFnAStar',
    'NavFnDijkstra',
    'ThetaStar',
    'Smac2D',
    'SmacHybrid',
)
PLANNER_LABELS = {
    'NavFnAStar': 'NavFn A*',
    'NavFnDijkstra': 'NavFn Dijkstra',
    'ThetaStar': 'Theta*',
    'Smac2D': 'Smac 2D',
    'SmacHybrid': 'Smac Hybrid',
}
ENVIRONMENT_LABELS = {
    'open_arena': 'Open arena',
    'narrow_aisles': 'Narrow aisles',
    'office_maze': 'Office maze',
}
COLORS = {
    'NavFnAStar': '#d1495b',
    'NavFnDijkstra': '#edae49',
    'ThetaStar': '#00798c',
    'Smac2D': '#30638e',
    'SmacHybrid': '#6f2dbd',
}


def font(size, bold=False):
    return ImageFont.truetype(FONT_BOLD if bold else FONT_REGULAR, size)


def centered_text(draw, box, text, selected_font, fill='#111111'):
    left, top, right, bottom = box
    bounds = draw.multiline_textbbox(
        (0, 0), text, font=selected_font, align='center'
    )
    width = bounds[2] - bounds[0]
    height = bounds[3] - bounds[1]
    draw.multiline_text(
        ((left + right - width) / 2, (top + bottom - height) / 2),
        text,
        font=selected_font,
        fill=fill,
        align='center',
    )


def _map_pixel(point, width, height):
    x, y = point
    return (
        (x + 6.0) / 12.0 * width,
        height - (y + 4.0) / 8.0 * height,
    )


def environment_overview():
    panel_width = 570
    panel_height = 380
    image = Image.new('RGB', (1840, 720), 'white')
    draw = ImageDraw.Draw(image)
    centered_text(
        draw,
        (0, 15, image.width, 80),
        'BA MÔI TRƯỜNG GAZEBO / NAV2 ĐỒNG NHẤT HÌNH HỌC',
        font(37, True),
    )
    for environment_index, environment in enumerate(ENVIRONMENTS):
        left = 55 + environment_index * 600
        top = 125
        map_image = Image.open(
            ROOT / 'src' / 'vacuum_robot_gazebo' / 'maps'
            / f'{environment}.pgm'
        ).convert('RGB')
        map_image = map_image.resize(
            (panel_width, panel_height), Image.Resampling.NEAREST
        )
        image.paste(map_image, (left, top))
        draw.rectangle(
            (left, top, left + panel_width, top + panel_height),
            outline='#202020',
            width=3,
        )
        with (
            ROOT / 'src' / 'adaptive_pivot_g2_benchmark' / 'config'
            / f'{environment}_scenarios.yaml'
        ).open(encoding='utf-8') as stream:
            scenarios = yaml.safe_load(stream)['scenarios']
        for scenario_index, scenario in enumerate(scenarios, start=1):
            start = _map_pixel(
                scenario['start'], panel_width, panel_height
            )
            goal = _map_pixel(
                scenario['goal'], panel_width, panel_height
            )
            start = (left + start[0], top + start[1])
            goal = (left + goal[0], top + goal[1])
            draw.line((start, goal), fill='#75a7bd', width=2)
            draw.ellipse(
                (
                    start[0] - 6,
                    start[1] - 6,
                    start[0] + 6,
                    start[1] + 6,
                ),
                fill='#167d3f',
                outline='white',
                width=2,
            )
            draw.polygon(
                (
                    (goal[0], goal[1] - 8),
                    (goal[0] - 7, goal[1] + 6),
                    (goal[0] + 7, goal[1] + 6),
                ),
                fill='#c1121f',
                outline='white',
            )
            draw.text(
                (start[0] + 7, start[1] - 12),
                str(scenario_index),
                font=font(14, True),
                fill='#111111',
                stroke_width=2,
                stroke_fill='white',
            )
        centered_text(
            draw,
            (
                left,
                top + panel_height + 15,
                left + panel_width,
                top + panel_height + 65,
            ),
            ENVIRONMENT_LABELS[environment],
            font(28, True),
        )
    centered_text(
        draw,
        (0, 610, image.width, 680),
        '● start   ▲ goal   — đường nối chỉ biểu diễn cặp bài toán, '
        'không phải đường planner',
        font(24),
        fill='#333333',
    )
    image.save(ASSETS / 'environment_overview.png')


def _raw_metrics():
    result = {}
    for environment in ENVIRONMENTS:
        with (
            ROOT / 'results'
            / f'planner_smoke_{environment}_20260723.csv'
        ).open(encoding='utf-8') as stream:
            rows = list(csv.DictReader(stream))
        for planner in PLANNERS:
            selected = [
                row for row in rows
                if row['planner'] == planner and row['method'] == 'raw'
            ]
            successful = [
                row for row in selected if row['success'] == 'True'
            ]
            result[(environment, planner)] = {
                'success_rate': len(successful) / len(selected),
                'path_length_m': statistics.fmean(
                    float(row['path_length_m']) for row in successful
                ),
                'clearance_m': statistics.fmean(
                    float(row['footprint_clearance_min_m'])
                    for row in successful
                ),
                'wall_time_ms': 1000.0 * statistics.fmean(
                    float(row['wall_time_s']) for row in successful
                ),
                'curvature_energy': statistics.fmean(
                    float(row['curvature_energy_1pm'])
                    for row in successful
                ),
            }
    return result


def _grouped_bars(
    draw,
    box,
    metrics,
    metric,
    title,
    maximum,
    display,
    transform=lambda value: value,
):
    left, top, right, bottom = box
    draw.rounded_rectangle(
        box, radius=12, fill='#fbfbfb', outline='#555555', width=2
    )
    centered_text(
        draw, (left, top + 5, right, top + 60), title, font(25, True)
    )
    chart_left = left + 62
    chart_right = right - 20
    chart_top = top + 78
    chart_bottom = bottom - 85
    draw.line(
        (chart_left, chart_top, chart_left, chart_bottom),
        fill='#222222',
        width=2,
    )
    draw.line(
        (chart_left, chart_bottom, chart_right, chart_bottom),
        fill='#222222',
        width=2,
    )
    group_width = (chart_right - chart_left) / len(ENVIRONMENTS)
    bar_width = group_width / (len(PLANNERS) + 1.5)
    for environment_index, environment in enumerate(ENVIRONMENTS):
        group_left = chart_left + environment_index * group_width
        for planner_index, planner in enumerate(PLANNERS):
            value = metrics[(environment, planner)][metric]
            bar_height = (
                transform(value) / maximum * (chart_bottom - chart_top - 25)
            )
            x0 = (
                group_left
                + (planner_index + 0.65) * bar_width
            )
            x1 = x0 + 0.82 * bar_width
            y0 = chart_bottom - bar_height
            draw.rectangle(
                (x0, y0, x1, chart_bottom),
                fill=COLORS[planner],
                outline='#333333',
                width=1,
            )
            centered_text(
                draw,
                (x0 - 15, y0 - 26, x1 + 15, y0 - 1),
                display(value),
                font(12, True),
            )
        centered_text(
            draw,
            (
                group_left,
                chart_bottom + 8,
                group_left + group_width,
                chart_bottom + 55,
            ),
            ENVIRONMENT_LABELS[environment].replace(' ', '\n'),
            font(17, True),
        )


def planner_pilot_comparison():
    metrics = _raw_metrics()
    image = Image.new('RGB', (1900, 1320), 'white')
    draw = ImageDraw.Draw(image)
    centered_text(
        draw,
        (0, 15, image.width, 80),
        'PILOT: SO SÁNH 5 GLOBAL PLANNER TRÊN 3 MAP',
        font(38, True),
    )
    boxes = (
        (45, 105, 925, 620),
        (975, 105, 1855, 620),
        (45, 655, 925, 1170),
        (975, 655, 1855, 1170),
    )
    _grouped_bars(
        draw,
        boxes[0],
        metrics,
        'success_rate',
        'Tỷ lệ lập kế hoạch thành công (raw)',
        1.0,
        lambda value: f'{100.0 * value:.0f}%',
    )
    _grouped_bars(
        draw,
        boxes[1],
        metrics,
        'clearance_m',
        'Clearance footprint tối thiểu trung bình (m)',
        0.35,
        lambda value: f'{value:.2f}',
    )
    _grouped_bars(
        draw,
        boxes[2],
        metrics,
        'curvature_energy',
        'Năng lượng độ cong raw, thang log10(1+E)',
        math.log10(
            1.0 + max(
                value['curvature_energy'] for value in metrics.values()
            )
        ),
        lambda value: f'{value:.1f}',
        transform=lambda value: math.log10(1.0 + value),
    )
    _grouped_bars(
        draw,
        boxes[3],
        metrics,
        'wall_time_ms',
        'Độ trễ action planner, thang log10(1+ms)',
        math.log10(
            1.0 + max(value['wall_time_ms'] for value in metrics.values())
        ),
        lambda value: f'{value:.1f}',
        transform=lambda value: math.log10(1.0 + value),
    )
    legend_left = 165
    for index, planner in enumerate(PLANNERS):
        x = legend_left + index * 330
        draw.rectangle(
            (x, 1220, x + 42, 1248),
            fill=COLORS[planner],
            outline='#333333',
        )
        draw.text(
            (x + 54, 1219),
            PLANNER_LABELS[planner],
            font=font(19, True),
            fill='#111111',
        )
    draw.text(
        (80, 1270),
        'Mỗi cột là trung bình có điều kiện trên 8 scenario; đây là pilot n=1, '
        'không phải kết luận thống kê.',
        font=font(18),
        fill='#444444',
    )
    image.save(ASSETS / 'planner_pilot_comparison.png')


def main():
    ASSETS.mkdir(parents=True, exist_ok=True)
    environment_overview()
    planner_pilot_comparison()
    for path in sorted(ASSETS.glob('*.png')):
        print(path)


if __name__ == '__main__':
    main()
