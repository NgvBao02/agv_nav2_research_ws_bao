#!/usr/bin/env python3
"""Extend journal v5 with existing five-method measurements; never launch ROS.

Editorial inputs: v5/manuscript.md, ver6/sections_ghi.md and the explicit
revisions below. Generated files are confined to journal/ver6.
"""
import csv
import hashlib
import json
import math
import re
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

import build_journal_sections_v5 as b

b.OUT = b.ROOT / 'Viet_bao/journal/ver6'
b.FIG = b.OUT / 'figures'
OLD = b.ROOT / 'Viet_bao/journal/ver5'
EXTRA = [
    'executed_curvature_energy_1pm', 'actual_p95_abs_jerk_mps3',
    'actual_max_angular_radps', 'actual_max_linear_mps',
    'actual_max_wheel_linear_mps', 'actual_max_abs_lateral_acceleration_mps2',
    'actual_max_abs_acceleration_mps2', 'actual_max_abs_angular_acceleration_radps2',
    'planned_pivot_marker_count', 'planned_pivot_total_angle_rad',
    'planned_point_count', 'physically_settled',
    'selected_start_anchor_adjustment_m', 'selected_goal_anchor_adjustment_m',
]


def fmt(x, digits=2):
    return f'{x:.{digits}f}'.replace('.', ',')


ORIGINAL_SCIENTIFIC_TEXT = b.add_scientific_text


def scientific_text(paragraph, text):
    forms = {
        'κt,max': [('κ', False), ('t,max', True)],
        'κmax': [('κ', False), ('max', True)],
        'Eκ,t': [('E', False), ('κ,t', True)],
        'emax': [('e', False), ('max', True)],
        'j95': [('j', False), ('95', True)],
        'ep': [('e', False), ('p', True)],
        'eψ': [('e', False), ('ψ', True)],
        '|vb|max': [('|v', False), ('b', True), ('|', False), ('max', True)],
        '|ω|max': [('|ω|', False), ('max', True)],
        '|a⊥|max': [('|a', False), ('⊥', True), ('|', False), ('max', True)],
        '|a|max': [('|a|', False), ('max', True)],
        '|α|max': [('|α|', False), ('max', True)],
        'vb': [('v', False), ('b', True)],
    }
    pattern = '(' + '|'.join(re.escape(k) for k in sorted(forms, key=len, reverse=True)) + ')'
    for part in re.split(pattern, text):
        if part in forms:
            for value, sub in forms[part]:
                run = paragraph.add_run(value)
                run.font.subscript = sub
                if not sub:
                    run.italic = True
        elif part:
            ORIGINAL_SCIENTIFIC_TEXT(paragraph, part)


def audit():
    """Require complete matched inputs and validate scalars against traces."""
    sys.path.insert(0, str(b.ROOT/'src/adaptive_pivot_g2_benchmark'))
    from adaptive_pivot_g2_benchmark.velocity_metrics import calculate_velocity_metrics
    issues = []
    for row in b.ROWS:
        d = b.DATA[row['route'], row['planner'], row['method']]
        for key in EXTRA:
            assert key in d and isinstance(d[key], (int, float, bool))
            assert math.isfinite(d[key]), (row['source'], key)
            row[key] = d[key]
        for key in ['collision_monitor_interventions', 'planned_footprint_collision_sample_count']:
            row[key] = int(row[key])
        assert d['success'] == row['success']
        assert d['state_trace_fields'] == ['time_s','x_m','y_m','yaw_rad','linear_mps','angular_radps']
        gt = d['ground_truth_state_trace']
        assert len(gt) > 2 and all(math.isfinite(v) for s in gt for v in s)
        metrics = calculate_velocity_metrics([(s[0], s[4], s[5]) for s in gt])
        for key in EXTRA:
            if not key.startswith('actual_'):
                continue
            translated = next(k for k in metrics if 'actual_'+k.replace('command_', '') == key)
            # Relative trace timestamps preserve derivatives except roundoff.
            assert math.isclose(d[key], metrics[translated], rel_tol=1e-7, abs_tol=1e-9), (key, d[key], metrics[translated])
        pts = np.asarray(d['executed_path_xy'], dtype=float)
        assert len(pts) >= 3
        a = np.linalg.norm(pts[1:-1]-pts[:-2], axis=1)
        c = np.linalg.norm(pts[2:]-pts[:-2], axis=1)
        bb = np.linalg.norm(pts[2:]-pts[1:-1], axis=1)
        cross = ((pts[1:-1,0]-pts[:-2,0])*(pts[2:,1]-pts[:-2,1])
                 -(pts[1:-1,1]-pts[:-2,1])*(pts[2:,0]-pts[:-2,0]))
        valid = a*bb*c > 1e-12
        energy = np.sum((2*cross[valid]/(a*bb*c)[valid])**2*(a[valid]+bb[valid])/2)
        assert math.isclose(energy, d['executed_curvature_energy_1pm'], rel_tol=1e-7, abs_tol=1e-8)
    for route in b.ROUTES:
        for planner in b.PLANNERS:
            rows = [r for r in b.ROWS if r['route']==route and r['planner']==planner]
            assert len(rows)==5 and len({r['raw_path_sha256'] for r in rows})==1
    b.fingerprint(Path(__file__))
    b.fingerprint(OLD/'manuscript.md')
    b.fingerprint(OLD/'PSTMO_JOURNAL_VER5_PHAN_II_III.docx')
    for name in ['velocity_metrics.py','compare_paths.py','execution_trial.py']:
        b.fingerprint(b.ROOT/'src/adaptive_pivot_g2_benchmark/adaptive_pivot_g2_benchmark'/name)
    return {'trials':125, 'matched_input_groups':25, 'complete_extra_fields':EXTRA,
            'velocity_traces_recomputed':125, 'executed_curvature_recomputed':125,
            'missing_metrics':issues, 'simulation_runs_added':0}


def paired(baseline, key, route=None, success=True):
    result=[]
    for rt in b.ROUTES if route is None else [route]:
        for planner in b.PLANNERS:
            p=next(r for r in b.ROWS if (r['route'],r['planner'],r['method'])==(rt,planner,'pstmo'))
            c=next(r for r in b.ROWS if (r['route'],r['planner'],r['method'])==(rt,planner,baseline))
            if not success or (p['success'] and c['success']):
                result.append(p[key]-c[key])
    return result


def make_tables():
    b.tables()
    heads, rows, widths = b.TABLES['baselines']
    rows[3][1] += '; hệ số giảm/tăng mẫu 1; giữ hướng đầu–cuối; tắt đi lùi'
    # These are explicit dataset settings; unspecified library defaults are
    # not retroactively inferred from an installed Nav2 release.
    for i, route in enumerate(b.ROUTES,1):
        b.TABLES[f'r{i:02}_extra']=(
            ['Phương án','L (m)','κmax (m⁻¹)','D (m)','emax (mm)','τ (ms)'],
            [[label]+[fmt(b.mean(route,m,k,succ)*scale,nd) for k,scale,nd,succ in [
                ('planned_path_length_m',1,3,False),('planned_max_abs_curvature_1pm',1,2,False),
                ('traveled_distance_m',1,3,True),('tracking_max_error_m',1000,1,True),
                ('smoothing_time_s',1000,1,False)]] for m,label in zip(b.METHODS,b.LABELS)],
            [.78,.48,.58,.48,.55,.51])
    b.TABLES['translation']=(['Phương án','κt,max (m⁻¹)','Eκ,t (m⁻¹)','N','Quay'],
        [[label,fmt(b.mean(None,m,'planned_translation_max_abs_curvature_1pm')),
          fmt(b.mean(None,m,'planned_translation_curvature_energy_1pm')),
          fmt(b.mean(None,m,'planned_point_count'),1),str(int(sum(b.values(None,m,'planned_pivot_marker_count'))))]
         for m,label in zip(b.METHODS,b.LABELS)], [.87,.68,.68,.62,.53])
    b.TABLES['execution_extra']=(['Phương án','Eκ,exec (m⁻¹)','j95 (m/s³)','ep (mm)','eψ (rad)'],
        [[label]+[fmt(b.mean(None,m,k,True)*scale,nd) for k,scale,nd in [
            ('executed_curvature_energy_1pm',1,3),('actual_p95_abs_jerk_mps3',1,3),
            ('final_position_error_m',1000,1),('final_yaw_error_rad',1,3)]]
         for m,label in zip(b.METHODS,b.LABELS)], [.86,.74,.66,.57,.55])
    b.TABLES['dynamics']=(['Phương án','|ω|max (rad/s)','|vb|max (m/s)','|a⊥|max (m/s²)','|a|max (m/s²)','|α|max (rad/s²)'],
        [[label]+[fmt(max(b.values(None,m,k)),3) for k in [
            'actual_max_angular_radps','actual_max_wheel_linear_mps',
            'actual_max_abs_lateral_acceleration_mps2','actual_max_abs_acceleration_mps2',
            'actual_max_abs_angular_acceleration_radps2']]
         for m,label in zip(b.METHODS,b.LABELS)], [.78,.52,.52,.52,.52,.52])
    b.TABLES['safety']=(['Phương án','τ (ms)','Dừng','Can thiệp','Mẫu va chạm'],
        [[label, '—' if m=='raw' else fmt(b.mean(None,m,'smoothing_time_s')*1000,2),
          f"{sum(b.values(None,m,'physically_settled'))}/25",
          str(sum(b.values(None,m,'collision_monitor_interventions'))),
          str(sum(b.values(None,m,'planned_footprint_collision_sample_count')))]
         for m,label in zip(b.METHODS,b.LABELS)], [.88,.58,.55,.62,.75])
    b.TABLES['all_pairs']=(['Đối chứng','n','ΔT (s)','Δe (mm)','ΔEκ,exec (m⁻¹)'],
        [[label, str(len(paired(m,'execution_time_s')))]+
         [fmt(np.mean(paired(m,k))*scale,3) for k,scale in [
             ('execution_time_s',1),('tracking_rmse_m',1000),('executed_curvature_energy_1pm',1)]]
         for m,label in zip(b.METHODS[:-1],b.LABELS[:-1])], [.86,.32,.67,.67,.86])
    # Failure table keeps the main and repeated cohorts explicitly distinct.
    heads,rows,widths=b.TABLES['failures']
    rows[:0]=[['R03 / Hybrid','P / KS','Vượt ngưỡng vị trí','0,102385'],
              ['R04 / Theta*','G / KS','Vượt ngưỡng vị trí','0,100234']]


def new_figures():
    original_savefig = b.savefig
    def compact_savefig(fig, name, *args, **kwargs):
        # Reduce unused space around the two-panel route figures without
        # reducing label fonts or changing the data/zoom window.
        if re.fullmatch(r'r\d\d_paths', name):
            fig.set_size_inches(b.COL, 3.65)
            fig.subplots_adjust(left=.16,right=.98,bottom=.115,top=.83,hspace=.62)
        elif re.fullmatch(r'r\d\d_metrics', name):
            fig.set_size_inches(b.COL, 3.25)
            fig.subplots_adjust(left=.17,right=.98,bottom=.16,top=.85,hspace=.22)
        original_savefig(fig, name, *args, **kwargs)
    b.savefig = compact_savefig
    try:
        b.figures()
    finally:
        b.savefig = original_savefig
    # Four baselines, actual matched-input differences; whiskers are ranges,
    # never confidence intervals inferred from nonreplicated cells.
    fig,axs=plt.subplots(3,1,figsize=(b.COL,4.0),sharex=True)
    for ax,key,scale,label,success in zip(axs,
        ['execution_time_s','tracking_rmse_m','planned_curvature_energy_1pm'],[1,1000,1],
        ['Δ thời gian (s)','Δ RMSE (mm)',r'Δ $E_{\kappa,ref}$ ($m^{-1}$)'],[True,True,False]):
        for j,m in enumerate(b.METHODS[:-1]):
            vals=np.array(paired(m,key,success=success))*scale
            avg=vals.mean()
            ax.vlines(j,vals.min(),vals.max(),color=b.COLORS[j],lw=1.4)
            ax.scatter(j,avg,color=b.COLORS[j],s=22,zorder=3)
            n=int(np.sum(vals < -1e-12))
            ax.text(j,1.01,f'{n}/{len(vals)} giảm',transform=ax.get_xaxis_transform(),
                    ha='center',va='bottom',fontsize=7)
        ax.axhline(0,color='black',lw=.6);ax.grid(axis='y',alpha=.25)
        ax.set_ylabel(label);ax.margins(y=.12);ax.set_xlim(-.55,3.55)
    axs[-1].set_xticks(range(4),b.LABELS[:-1],rotation=12)
    fig.subplots_adjust(left=.23,right=.98,bottom=.12,top=.95,hspace=.45)
    b.savefig(fig,'matrix_delta','Bốn đối chứng; chấm trung bình, đoạn thẳng min–max của chênh lệch ghép cặp. Không phải CI.')
    fig,axs=plt.subplots(3,1,figsize=(b.COL,4.5),gridspec_kw={'height_ratios':[1.25,1,1]})
    rt=b.ROUTES[1]
    b.map_axes(axs[0]);axs[0].set(xlim=(.7,2.8),ylim=(.4,1.4))
    for j,m in enumerate(b.METHODS):
        d=b.DATA[rt,'ThetaStar',m];path=np.array(d['selected_path_xy']);gt=np.array(d['ground_truth_state_trace'])
        axs[0].plot(path[:,0],path[:,1],ls='--',color=b.COLORS[j],lw=.7)
        axs[0].plot(gt[:,1],gt[:,2],color=b.COLORS[j],lw=1,label=b.LABELS[j])
        ax=axs[1 if j<3 else 2]
        ax.plot(gt[:,0],gt[:,5],color=b.COLORS[j],lw=.7,label=b.LABELS[j])
    axs[0].legend(ncol=3,loc='lower center',bbox_to_anchor=(.5,1),frameon=False,fontsize=7,columnspacing=.7)
    for ax in axs[1:]:
        ax.set_ylabel('ω (rad/s)');ax.grid(alpha=.25)
        ax.set_ylim(-.28,.28);ax.set_xlim(0,65)
        ax.legend(loc='lower center',bbox_to_anchor=(.5,1.02),ncol=3,frameon=False,fontsize=7)
    axs[2].set_xlabel('Thời gian mô phỏng tương đối (s)')
    fig.subplots_adjust(left=.18,right=.98,bottom=.10,top=.89,hspace=.85)
    b.savefig(fig,'execution_trace','R02/Theta*: năm phương án, tín hiệu góc không lọc; cùng thang thời gian.',
               [r['source'] for r in b.ROWS if r['route']==rt and r['planner']=='ThetaStar'])


def all_method_values(route,key,scale=1,digits=3):
    return '; '.join(fmt(b.mean(route,m,key,True)*scale,digits) for m in b.METHODS)


ROUTE_COMMENTS=[
    'PSTMO giảm sai số cực đại cùng với RMSE, nhưng độ giật không thấp nhất. Vì vậy, lợi ích về hình dạng quỹ đạo không đồng nghĩa giảm đồng thời mọi biến thiên theo thời gian.',
    'PSTMO có RMSE cao nhất ở tuyến này, trong khi sai số cực đại thấp hơn đường gốc và Constrained nhưng cao hơn Simple. Hai chỉ số sai số mô tả hai khía cạnh khác nhau: độ lệch duy trì dọc đường và độ lệch lớn nhất tại một vị trí.',
    'Độ cong cực đại của PSTMO lớn hơn Simple dù tích phân bình phương độ cong nhỏ hơn. Độ giật cũng cao hơn các đối chứng trong tập lượt đạt. Các kết quả này không hỗ trợ kết luận PSTMO luôn tạo chuyển động trơn hơn theo mọi tiêu chí.',
    'PSTMO giảm sai số bám đường cực đại so với cả bốn đối chứng, nhưng sai số vị trí cuối lớn hơn các phương án còn lại trong tập lượt đạt. Khả năng bám đường và độ chính xác dừng tại đích cần được đánh giá riêng.',
    'PSTMO có sai số cực đại và tích phân bình phương độ cong thực thi thấp nhất ở mức trung bình, nhưng sai số vị trí cuối cao nhất. Lợi ích trong đoạn di chuyển không bảo đảm cải thiện tương ứng khi dừng tại đích.',
]


def make_source():
    source=(OLD/'manuscript.md').read_text()
    head=source.split('## G. Tổng hợp theo tuyến')[0]
    # Replace five narrow PSTMO–Constrained tables with five-method extensions.
    for i,route in enumerate(b.ROUTES,1):
        note = ('L: chiều dài đường tham chiếu; κmax: độ cong tuyệt đối cực đại chưa lấy mẫu lại; '
                'D: quãng đường thực thi; emax: sai số bám đường cực đại; τ: thời lượng hậu xử lý. '
                if i == 1 else 'Ký hiệu như Bảng VI. ')
        note += 'L, κmax và τ dùng năm lượt; D và emax chỉ dùng lượt đạt.'
        head=re.sub(rf'@table r{i:02}_paired [^\n]+',
            f'@table r{i:02}_extra Chiều dài, độ cong cực đại và chỉ số bổ sung tại R{i:02}\n\n'
            '@note '+note,head)
    # Existing references to removed pair tables become descriptive statements.
    for old,new in [('Bảng VI cho thấy','So sánh ghép cặp cho thấy'),
                    ('Bảng VIII cho thấy','So sánh theo bộ lập kế hoạch cho thấy'),
                    ('Bảng XII cho thấy','So sánh theo bộ lập kế hoạch cho thấy'),
                    ('Bảng XIV cho thấy','So sánh theo bộ lập kế hoạch cho thấy')]:
        head=head.replace(old,new)
    head=head.replace('Do các trung bình dùng số lượt khác nhau, Bảng X cung cấp thêm chênh lệch trên bốn cặp có cả hai phương án đạt. Cặp Smac Hybrid được để trống trong phép so sánh liên tục.',
        'Các trung bình này dùng số lượt khác nhau. Vì vậy, so sánh ghép cặp với PSTMO chỉ dùng bốn đầu vào có cả hai phương án đạt; lượt Smac Hybrid được giữ trong tỷ lệ thành công. Bảng X bổ sung chỉ số hình học và thực thi của cả năm phương án.')
    head=head.replace('Tuy nhiên, So sánh ghép cặp', 'Tuy nhiên, so sánh ghép cặp')
    head=head.replace('Bảng X xác định ngoại lệ với Constrained', 'Ngoại lệ khi so với Constrained')
    head=head.replace('các thất bại của bộ thử nghiệm lặp được báo riêng ở Mục III-H',
                      'các thất bại của bộ thử nghiệm lặp được báo riêng ở Mục III-I')
    for i,route in enumerate(b.ROUTES,1):
        marker=f'@table r{i:02}_extra '
        start=head.index(marker)
        next_section=head.index('### 3)',start)
        para=('Theo thứ tự đường gốc, Simple, Savitzky–Golay, Constrained và PSTMO, '
              f"Eκ,exec lần lượt là {all_method_values(route,'executed_curvature_energy_1pm')} m⁻¹; "
              f"phân vị 95% của trị tuyệt đối độ giật lần lượt là {all_method_values(route,'actual_p95_abs_jerk_mps3')} m/s³. "+
              ROUTE_COMMENTS[i-1]+'\n\n')
        head=head[:next_section]+para+head[next_section:]
    anchor='Các chỉ số bổ sung gồm sai số đích, quãng đường thực thi, phân vị 95%'
    idx=head.index(anchor)
    # Keep original definitions, extend them immediately before R01.
    end=head.index('## B. R01',idx)
    head=head[:end]+('Chiều dài L được tính bằng tổng khoảng cách giữa các điểm đường tham chiếu; '
        'D là tổng quãng đường từ các vị trí robot trước bước xử lý quỹ đạo. Sai số cực đại emax dùng cùng phép đo khoảng cách như RMSE. '
        'Sai số cuối ep và eψ lần lượt là khoảng cách tới vị trí đích và trị tuyệt đối sai lệch hướng đã quy về [0; π]. '
        'Các chỉ số hình học được báo trên toàn bộ 25 đường của mỗi phương án; các trung bình thực thi dùng tập lượt đạt và ghi rõ mẫu số.\n\n'
        'Độ cong cực đại κmax là cực đại trị tuyệt đối độ cong ước lượng trên các điểm đầu ra chưa lấy mẫu lại. '
        'Để kiểm tra ảnh hưởng lấy mẫu, phần tịnh tiến được tách tại các tư thế quay tại chỗ và lấy mẫu lại với bước 0,05 m, '
        'cho các chỉ số κt,max và Eκ,t. Dấu quay tại chỗ được nhận diện khi dịch chuyển không quá 0,0001 m và thay đổi hướng ít nhất 5°. '
        'Đây là dấu trong đường tham chiếu, không phải số lần robot thực sự xoay trong quá trình điều khiển.\n\n'
        'Toàn bộ 125 bản ghi có đủ chỉ số quỹ đạo thực thi, độ giật và chuỗi trạng thái. '
        'Các chỉ số bổ sung được trích cùng nguồn với thời gian và RMSE. Vận tốc bánh tương đương được suy từ v ± bω/2 với b = 0,2548 m; '
        'gia tốc ngang được tính bằng |vω|. Gia tốc và độ giật dùng sai phân với bước thời gian hợp lệ từ 0,0001 đến 0,5 s. '
        'Cực đại của các tín hiệu chỉ mô tả những mẫu đã ghi, chưa chứng minh ràng buộc liên tục giữa các mẫu.\n\n')+head[end:]
    # Quantitative terminal comparisons for each route are directly recoverable.
    for i,route in enumerate(b.ROUTES,1):
        target=f'## {chr(ord("B")+i)}. R{i+1:02}' if i<5 else None
        end=head.index(target) if target else len(head)
        extra=('Sai số vị trí cuối trung bình của năm phương án theo thứ tự trên là '
               f"{all_method_values(route,'final_position_error_m',1000,1)} mm; sai số hướng tương ứng là "
               f"{all_method_values(route,'final_yaw_error_rad',1,3)} rad, chỉ tính các lượt đạt.\n\n")
        head=head[:end]+extra+head[end:]
    tail=(b.OUT/'sections_ghi.md').read_text()
    refs=source[source.index('# TÀI LIỆU THAM KHẢO'):]
    source=head+tail+'\n\n'+refs
    # Keep tables and short analysis near their first mention, and avoid
    # pushing a tall figure into a column with a large unused lower half.
    chunks=re.split(r'\n\s*\n', source.strip())
    for figure, after_note in [('r02_paths','r02'),('r05_paths','r05_extra'),
                               ('repeat_blocks','secondary')]:
        index=next(j for j,c in enumerate(chunks) if c.startswith('@figure '+figure+' '))
        chunk=chunks.pop(index)
        index=next(j for j,c in enumerate(chunks) if c.startswith('@table '+after_note+' '))
        assert chunks[index+1].startswith('@note ')
        chunks.insert(index+2,chunk)
    source='\n\n'.join(chunks)+'\n'
    tn={k:i for i,k in enumerate(re.findall(r'^@table (\S+)',source,re.M),1)}
    fn={k:i for i,k in enumerate(re.findall(r'^@figure (\S+)',source,re.M),1)}
    source=re.sub(r'\{table:([^}]+)\}',lambda m:b.roman(tn[m[1]]),source)
    source=re.sub(r'\{figure:([^}]+)\}',lambda m:str(fn[m[1]]),source)
    (b.OUT/'manuscript.md').write_text(source)
    return tn,fn


def export_analysis(audit_report):
    folder=b.OUT/'analysis';folder.mkdir(exist_ok=True)
    with (folder/'all_125_metrics.csv').open('w',newline='',encoding='utf-8-sig') as f:
        writer=csv.DictWriter(f,fieldnames=list(b.ROWS[0]));writer.writeheader();writer.writerows(b.ROWS)
    comparisons=[]
    numeric=[k for k,v in b.ROWS[0].items() if isinstance(v,(float,int)) and not isinstance(v,bool)]
    summaries=[]
    for route in [None]+b.ROUTES:
        for m in b.METHODS:
            for key in numeric:
                succ=not (key.startswith('planned_') or key in ['smoothing_time_s','collision_monitor_interventions'] or key.startswith('selected_'))
                vals=b.values(route,m,key,succ)
                summaries.append({'route':route or 'ALL','method':m,'metric':key,'success_only':succ,
                                  'n':len(vals),'mean':float(np.mean(vals)),
                                  'min':float(min(vals)),'max':float(max(vals))})
        for m in b.METHODS[:-1]:
            for key in numeric:
                succ=not (key.startswith('planned_') or key=='smoothing_time_s' or key.startswith('selected_'))
                vals=paired(m,key,route,succ)
                comparisons.append({'route':route or 'ALL','baseline':m,'metric':key,'success_only':succ,
                    'n':len(vals),'mean_delta':float(np.mean(vals)),'min_delta':float(min(vals)),
                    'max_delta':float(max(vals)),'pstmo_lower':sum(v < -1e-12 for v in vals),
                    'equal':sum(abs(v)<=1e-12 for v in vals),'pstmo_higher':sum(v>1e-12 for v in vals)})
    for name,rows in [('summary_metrics',summaries),('pairwise_comparisons',comparisons)]:
        with (folder/(name+'.csv')).open('w',newline='',encoding='utf-8-sig') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    (folder/'data_audit.json').write_text(json.dumps(audit_report,ensure_ascii=False,indent=2)+'\n')
    return comparisons


def main():
    b.OUT.mkdir(exist_ok=True);b.FIG.mkdir(exist_ok=True)
    b.add_scientific_text=scientific_text
    report=audit();make_tables();new_figures();tn,fn=make_source();export_analysis(report)
    b.fingerprint(b.OUT/'sections_ghi.md')
    original_create=b.create_doc
    def create():
        doc=original_create();doc.core_properties.subject='Phần II và III của bản thảo journal phiên bản 6'
        return doc
    b.create_doc=create
    b.manuscript()
    old=b.OUT/'PSTMO_JOURNAL_VER5_PHAN_II_III.docx'
    new=b.OUT/'PSTMO_JOURNAL_VER6_PHAN_II_III.docx';old.replace(new)
    path=b.OUT/'provenance.json';p=json.loads(path.read_text());p['docx']=new.name
    p['table_order']=tn;p['figure_order']=fn
    p['figures']={k:v for k,v in p['figures'].items() if k in fn}
    p['data_audit']=report;p['docx_sha256']=hashlib.sha256(new.read_bytes()).hexdigest()
    path.write_text(json.dumps(p,ensure_ascii=False,indent=2)+'\n')
    (b.OUT/'table_values.json').write_text(json.dumps({k:b.TABLES[k] for k in tn},ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({'output':str(new),'counts':p['counts']},ensure_ascii=False))


if __name__=='__main__':
    main()
