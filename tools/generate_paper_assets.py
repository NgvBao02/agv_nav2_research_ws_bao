#!/usr/bin/env python3

"""Generate reproducible paper figures with Pillow only."""

import csv
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[1]
ASSETS = ROOT / 'docs' / 'paper_assets'
FONT_REGULAR = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
FONT_BOLD = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'

METHODS = [
    'raw',
    'simple',
    'savitzky_golay',
    'constrained',
    'pivot_g2',
    'adaptive_hybrid',
]
LABELS = {
    'raw': 'Raw',
    'simple': 'Simple',
    'savitzky_golay': 'Savitzky–Golay',
    'constrained': 'Constrained',
    'pivot_g2': 'Pivot–G2',
    'adaptive_hybrid': 'Hybrid',
}
COLORS = {
    'raw': '#d62728',
    'simple': '#ff9f1c',
    'savitzky_golay': '#00a6d6',
    'constrained': '#2ca02c',
    'pivot_g2': '#d336c5',
    'adaptive_hybrid': '#2457ff',
}


def font(size, bold=False):
    return ImageFont.truetype(FONT_BOLD if bold else FONT_REGULAR, size)


def centered_text(draw, box, text, selected_font, fill='#111111'):
    left, top, right, bottom = box
    bounds = draw.textbbox((0, 0), text, font=selected_font)
    width = bounds[2] - bounds[0]
    height = bounds[3] - bounds[1]
    draw.text(
        ((left + right - width) / 2, (top + bottom - height) / 2),
        text,
        font=selected_font,
        fill=fill,
    )


def arrow(draw, start, end, fill='#4f5d75', width=5):
    draw.line([start, end], fill=fill, width=width)
    x1, y1 = start
    x2, y2 = end
    length = max(((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5, 1.0)
    ux = (x2 - x1) / length
    uy = (y2 - y1) / length
    px = -uy
    py = ux
    tip = (x2, y2)
    base = (x2 - 18 * ux, y2 - 18 * uy)
    draw.polygon(
        [tip, (base[0] + 9 * px, base[1] + 9 * py),
         (base[0] - 9 * px, base[1] - 9 * py)],
        fill=fill,
    )


def architecture_figure():
    image = Image.new('RGB', (1800, 820), 'white')
    draw = ImageDraw.Draw(image)
    centered_text(
        draw, (0, 20, 1800, 90),
        'KIẾN TRÚC THỬ NGHIỆM ROS 2 / NAV2', font(42, True)
    )
    boxes = [
        ((60, 180, 340, 350), 'ThetaStar\n+ costmap'),
        ((440, 130, 750, 400), '6 phương pháp\nRaw · Simple · SG\nConstrained\nPivot–G2 · Hybrid'),
        ((850, 180, 1130, 350), 'Controller chung\nmaneuver-aware'),
        ((1230, 130, 1550, 400), 'Robot vi sai\n0,44 × 0,34 m\nA1M8 · BNO055'),
    ]
    fills = ['#e8f1fb', '#fff2cc', '#e2f0d9', '#fce4d6']
    for (box, label), fill in zip(boxes, fills):
        draw.rounded_rectangle(box, radius=22, fill=fill, outline='#36454f', width=4)
        centered_text(draw, box, label, font(29, True))
    for first, second in zip(boxes, boxes[1:]):
        arrow(
            draw,
            (first[0][2] + 15, (first[0][1] + first[0][3]) / 2),
            (second[0][0] - 15, (second[0][1] + second[0][3]) / 2),
        )
    draw.rounded_rectangle(
        (440, 520, 1550, 700), radius=22, fill='#f3f3f3',
        outline='#606060', width=3
    )
    centered_text(
        draw, (440, 520, 1550, 700),
        'Ground truth Gazebo + TF/odom/scan/cmd_vel\n'
        '→ success, clearance, độ cong, thời gian, sai số bám',
        font(29, True)
    )
    arrow(draw, (1390, 405), (1390, 505))
    arrow(draw, (1000, 505), (1000, 410))
    image.save(ASSETS / 'figure_1_system_architecture.png', quality=95)


def dashed_polyline(draw, points, fill, width=4, dash=12):
    for first, last in zip(points, points[1:]):
        dx = last[0] - first[0]
        dy = last[1] - first[1]
        length = max((dx * dx + dy * dy) ** 0.5, 1.0)
        steps = max(int(length / dash), 1)
        for index in range(0, steps, 2):
            a = index / steps
            b = min((index + 1) / steps, 1.0)
            draw.line(
                [
                    (first[0] + a * dx, first[1] + a * dy),
                    (first[0] + b * dx, first[1] + b * dy),
                ],
                fill=fill,
                width=width,
            )


def map_panel(canvas, box, bounds, paths, title):
    left, top, right, bottom = box
    width = right - left
    height = bottom - top
    map_image = Image.open(
        ROOT / 'src/vacuum_robot_gazebo/maps/research_warehouse.pgm'
    ).convert('RGB')
    map_width, map_height = map_image.size
    resolution = 0.05
    origin_x, origin_y = -6.0, -4.0

    def map_pixel(x, y):
        return (
            (x - origin_x) / resolution,
            map_height - (y - origin_y) / resolution,
        )

    x_min, x_max, y_min, y_max = bounds
    crop_left, crop_bottom = map_pixel(x_min, y_min)
    crop_right, crop_top = map_pixel(x_max, y_max)
    crop = map_image.crop((crop_left, crop_top, crop_right, crop_bottom))
    crop = crop.resize((width, height), Image.Resampling.NEAREST)
    canvas.paste(crop, (left, top))
    draw = ImageDraw.Draw(canvas)
    draw.rectangle(box, outline='#222222', width=3)
    draw.rectangle((left + 8, top + 8, left + 470, top + 54), fill='white')
    draw.text((left + 18, top + 12), title, font=font(25, True), fill='#111111')

    def panel_point(x, y):
        return (
            left + (x - x_min) / (x_max - x_min) * width,
            bottom - (y - y_min) / (y_max - y_min) * height,
        )

    for method in METHODS:
        points = [panel_point(point['x'], point['y']) for point in paths[method]]
        if method == 'pivot_g2':
            draw.line(points, fill=COLORS[method], width=9, joint='curve')
        elif method == 'adaptive_hybrid':
            dashed_polyline(draw, points, COLORS[method], width=5, dash=10)
        else:
            draw.line(points, fill=COLORS[method], width=5, joint='curve')
    start = panel_point(paths['raw'][0]['x'], paths['raw'][0]['y'])
    goal = panel_point(paths['raw'][-1]['x'], paths['raw'][-1]['y'])
    draw.ellipse(
        (start[0] - 10, start[1] - 10, start[0] + 10, start[1] + 10),
        fill='#111111', outline='white', width=3
    )
    draw.polygon(
        [(goal[0], goal[1] - 13), (goal[0] - 12, goal[1] + 11),
         (goal[0] + 12, goal[1] + 11)],
        fill='#111111', outline='white'
    )


def path_comparison_figure():
    with (ASSETS / 'lower_left_diagonal_paths.json').open(encoding='utf-8') as stream:
        paths = json.load(stream)['paths']
    image = Image.new('RGB', (2100, 1160), 'white')
    draw = ImageDraw.Draw(image)
    centered_text(
        draw, (0, 10, 2100, 80),
        'SO SÁNH ĐƯỜNG ĐI TRÊN CÙNG RAW PATH – LOWER_LEFT_DIAGONAL',
        font(38, True)
    )
    map_panel(image, (55, 110, 1005, 875), (-6.0, 6.0, -4.0, 4.0), paths,
              'Toàn bộ bản đồ kho')
    map_panel(image, (1095, 110, 2045, 875), (-5.35, -1.55, -3.35, -0.05), paths,
              'Phóng to vùng chuyển hướng')
    legend_y = 925
    x = 80
    for method in METHODS:
        draw.line((x, legend_y, x + 70, legend_y), fill=COLORS[method], width=8)
        draw.text((x + 82, legend_y - 18), LABELS[method], font=font(23, True), fill='#111111')
        x += 320
    draw.text(
        (80, 1020),
        '● điểm đầu; ▲ điểm đích. Hybrid (nét xanh đứt) trùng Pivot–G2 trong '
        'tình huống này; marker pivot được thực thi bởi controller chung.',
        font=font(25), fill='#222222'
    )
    image.save(ASSETS / 'figure_2_path_comparison.png', quality=95)


def grouped_bars(draw, box, values, title, unit, maximum, decimals=3):
    left, top, right, bottom = box
    draw.rectangle(box, outline='#555555', width=2)
    centered_text(draw, (left, top + 5, right, top + 55), title, font(27, True))
    chart_top = top + 75
    chart_bottom = bottom - 115
    base_y = chart_bottom
    draw.line((left + 75, chart_top, left + 75, base_y), fill='#222222', width=3)
    draw.line((left + 75, base_y, right - 25, base_y), fill='#222222', width=3)
    span = right - left - 120
    bar_width = int(span / len(METHODS) * 0.62)
    for index, method in enumerate(METHODS):
        center = left + 90 + (index + 0.5) * span / len(METHODS)
        bar_height = values[method] / maximum * (base_y - chart_top - 10)
        draw.rectangle(
            (center - bar_width / 2, base_y - bar_height,
             center + bar_width / 2, base_y),
            fill=COLORS[method], outline='#333333', width=2
        )
        value_text = f'{values[method]:.{decimals}f}'
        centered_text(
            draw,
            (center - 80, base_y - bar_height - 38, center + 80, base_y - bar_height),
            value_text, font(18, True)
        )
        centered_text(
            draw, (center - 90, base_y + 8, center + 90, base_y + 55),
            LABELS[method].replace('Savitzky–Golay', 'SG'), font(18, True)
        )
    draw.text((left + 14, chart_top), unit, font=font(19), fill='#333333')


def offline_metrics_figure():
    rows = list(csv.DictReader(
        (ROOT / 'results/fair_batch_v4b_hybrid_20260723.csv').open(encoding='utf-8')
    ))
    energy = {}
    clearance = {}
    for method in METHODS:
        selected = [row for row in rows if row['method'] == method]
        energy[method] = sum(
            float(row['translation_curvature_energy_1pm']) for row in selected
        ) / len(selected)
        clearance[method] = sum(
            float(row['footprint_clearance_min_m']) for row in selected
        ) / len(selected)
    image = Image.new('RGB', (2100, 980), 'white')
    draw = ImageDraw.Draw(image)
    centered_text(
        draw, (0, 5, 2100, 75),
        'KẾT QUẢ OFFLINE – TRUNG BÌNH 12 TÌNH HUỐNG', font(39, True)
    )
    grouped_bars(
        draw, (45, 100, 1025, 900), energy,
        'Năng lượng độ cong tịnh tiến', '∫κ²ds (1/m)', 15.0, 2
    )
    grouped_bars(
        draw, (1075, 100, 2055, 900), clearance,
        'Clearance nhỏ nhất của footprint', 'm', 0.32, 3
    )
    image.save(ASSETS / 'figure_3_offline_metrics.png', quality=95)


def dot_panel(draw, box, means, deviations, title, unit, minimum, maximum, digits):
    left, top, right, bottom = box
    draw.rectangle(box, outline='#555555', width=2)
    centered_text(draw, (left, top + 6, right, top + 60), title, font(27, True))
    axis_left = left + 250
    axis_right = right - 70
    axis_top = top + 100
    row_height = (bottom - axis_top - 70) / len(METHODS)
    draw.line((axis_left, bottom - 60, axis_right, bottom - 60), fill='#222222', width=3)
    for tick in range(6):
        value = minimum + tick * (maximum - minimum) / 5
        x = axis_left + tick / 5 * (axis_right - axis_left)
        draw.line((x, bottom - 65, x, bottom - 52), fill='#222222', width=2)
        centered_text(draw, (x - 60, bottom - 50, x + 60, bottom - 5),
                      f'{value:.{digits}f}', font(17))
    for index, method in enumerate(METHODS):
        y = axis_top + (index + 0.5) * row_height
        draw.text((left + 18, y - 16), LABELS[method], font=font(20, True), fill='#222222')
        mean_x = axis_left + (means[method] - minimum) / (maximum - minimum) * (
            axis_right - axis_left
        )
        low_x = axis_left + (means[method] - deviations[method] - minimum) / (
            maximum - minimum
        ) * (axis_right - axis_left)
        high_x = axis_left + (means[method] + deviations[method] - minimum) / (
            maximum - minimum
        ) * (axis_right - axis_left)
        draw.line((low_x, y, high_x, y), fill=COLORS[method], width=5)
        draw.line((low_x, y - 8, low_x, y + 8), fill=COLORS[method], width=4)
        draw.line((high_x, y - 8, high_x, y + 8), fill=COLORS[method], width=4)
        draw.ellipse((mean_x - 9, y - 9, mean_x + 9, y + 9), fill=COLORS[method])
    draw.text((axis_left, top + 66), unit, font=font(18), fill='#333333')


def closed_loop_figure():
    with (
        ROOT / 'results/execution_matrix_v7_clean_repeated_20260723/'
        'lower_left_diagonal_summary.json'
    ).open(encoding='utf-8') as stream:
        aggregates = json.load(stream)['aggregates']
    time_mean = {method: aggregates[method]['execution_time_s_mean'] for method in METHODS}
    time_sd = {method: aggregates[method]['execution_time_s_stdev'] for method in METHODS}
    rmse_mean = {method: aggregates[method]['tracking_rmse_m_mean'] for method in METHODS}
    rmse_sd = {method: aggregates[method]['tracking_rmse_m_stdev'] for method in METHODS}
    image = Image.new('RGB', (2100, 1000), 'white')
    draw = ImageDraw.Draw(image)
    centered_text(
        draw, (0, 5, 2100, 75),
        'KẾT QUẢ CLOSED-LOOP – MEAN ± SD, n = 3', font(39, True)
    )
    dot_panel(
        draw, (45, 100, 1025, 910), time_mean, time_sd,
        'Thời gian hoàn thành', 'giây (trục 61–67 s)', 61.0, 67.0, 1
    )
    dot_panel(
        draw, (1075, 100, 2055, 910), rmse_mean, rmse_sd,
        'Sai số bám RMS', 'm', 0.0, 0.025, 3
    )
    image.save(ASSETS / 'figure_4_closed_loop_metrics.png', quality=95)


def main():
    ASSETS.mkdir(parents=True, exist_ok=True)
    architecture_figure()
    path_comparison_figure()
    offline_metrics_figure()
    closed_loop_figure()
    for path in sorted(ASSETS.glob('figure_*.png')):
        print(path)


if __name__ == '__main__':
    main()
