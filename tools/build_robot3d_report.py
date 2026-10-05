#!/usr/bin/env python3
"""Vietnamese, evidence-first 3D robot report with fixed, checked page layouts."""
import csv,json,math,html,hashlib
from pathlib import Path
import numpy as np
from PIL import Image as PILImage
from reportlab.pdfgen import canvas
from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle
from reportlab.platypus import Paragraph,Table,TableStyle
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib.utils import ImageReader

ROOT=Path(__file__).resolve().parents[1];A=ROOT/'docs/robot_3d_report'
PDF=ROOT/'docs/BAO_CAO_MO_HINH_3D_STEP_URDF_GAZEBO_RVIZ2.pdf'
for name,file in [('DV','DejaVuSans.ttf'),('DVB','DejaVuSans-Bold.ttf'),('DVI','DejaVuSans-Oblique.ttf')]:
    pdfmetrics.registerFont(TTFont(name,'/usr/share/fonts/truetype/dejavu/'+file))
pdfmetrics.registerFontFamily('DV',normal='DV',bold='DVB',italic='DVI',boldItalic='DVB')
W,H=595.276,841.89;M=42;CW=W-2*M
NAVY=colors.HexColor('#17334b');TEAL=colors.HexColor('#087f8c');INK=colors.HexColor('#263746');MUTED=colors.HexColor('#546777');PALE=colors.HexColor('#edf4f6');ORANGE=colors.HexColor('#b7631b')
styles={
 'body':ParagraphStyle('body',fontName='DV',fontSize=10.1,leading=14.4,textColor=INK,spaceAfter=8),
 'small':ParagraphStyle('small',fontName='DV',fontSize=8.5,leading=11.5,textColor=MUTED,spaceAfter=6),
 'caption':ParagraphStyle('caption',fontName='DV',fontSize=8.1,leading=11,textColor=MUTED,spaceAfter=9),
 'title':ParagraphStyle('title',fontName='DVB',fontSize=21,leading=26,textColor=NAVY,spaceAfter=11),
 'h2':ParagraphStyle('h2',fontName='DVB',fontSize=12,leading=16,textColor=TEAL,spaceAfter=7),
 'cell':ParagraphStyle('cell',fontName='DV',fontSize=8.4,leading=11.4,textColor=INK),
 'headcell':ParagraphStyle('headcell',fontName='DVB',fontSize=8.4,leading=11.4,textColor=colors.white),
}
def clean(s):return str(s).replace('\u2013','-').replace('\u2014','-').replace('\u2011','-')
def e(s):return html.escape(clean(s))
def j(n):return json.loads((A/n).read_text())
G=j('geometry.json');AUD=j('cad_audit.json');TOP=j('step_topology.json');MASS=j('mass_properties.json');SUM=j('motion/summary.json');RATES=j('motion/rates.json');WM=j('warehouse_metrics.json');PROV=j('provenance.json')
SM={x['case']:x for x in SUM};TRIALS=j('motion/trials.json')
LABEL={'forward':'Tiến thẳng','reverse':'Lùi thẳng','pivot_ccw':'Quay trái tại chỗ','pivot_cw':'Quay phải tại chỗ','arc_left':'Chạy cung trái','arc_right':'Chạy cung phải','brake':'Dừng từ 0,30 m/s'}
def num(x,d=3):return f'{0. if abs(x)<.5*10**(-d) else x:.{d}f}'.replace('.',',')

class Report:
    def __init__(self,path,total=0,prior=None):
        self.c=canvas.Canvas(str(path),pagesize=(W,H));self.c.setTitle('Mô hình 3D robot vi sai - STEP, URDF, Gazebo và RViz2');self.c.setAuthor('Hồ sơ nghiên cứu agv_nav2_research_ws_bao')
        self.page=0;self.fig=0;self.total=total;self.pages=[];self.figures=[];self.prior=prior;self.sections=[]
    def new(self,title,section='',chapter=False):
        if self.page:self.c.showPage()
        self.page+=1;self.y=H-69;self.pages.append(dict(page=self.page,title=title,section=section))
        self.c.setFillColor(TEAL);self.c.rect(M,H-33,30,3,fill=1,stroke=0)
        self.c.setFont('DVB',8);self.c.drawString(M+40,H-33,'ROBOT 3D  /  HỒ SƠ MÔ HÌNH VÀ CHUYỂN ĐỘNG')
        self.c.setFont('DV',7.5);self.c.setFillColor(MUTED);self.c.drawRightString(W-M,H-33,section)
        self.c.setStrokeColor(colors.HexColor('#d7e1e5'));self.c.line(M,40,W-M,40)
        self.c.setFont('DV',7.4);self.c.drawString(M,27,'Dữ liệu ngày 05/10/2026  |  Mô phỏng, không thay thế thử nghiệm robot thật')
        self.c.drawRightString(W-M,27,f'{self.page} / {self.total}' if self.total else str(self.page))
        self.c.bookmarkPage('p'+str(self.page))
        self.c.addOutlineEntry(clean(title),'p'+str(self.page),level=0 if chapter else 1)
        if chapter:self.sections.append(dict(title=title,page=self.page))
        self.p(title,'title')
    def check(self,h):
        if self.y-h<53:raise ValueError(f'Page {self.page} overflow: y={self.y:.1f}, required={h:.1f}, title={self.pages[-1]["title"]}')
    def p(self,text,kind='body'):
        ob=Paragraph(clean(text),styles[kind]);_,h=ob.wrap(CW,1000);self.check(h+styles[kind].spaceAfter);ob.drawOn(self.c,M,self.y-h);self.y-=h+styles[kind].spaceAfter
    def table(self,headers,rows,widths=None):
        data=[[Paragraph(e(x),'headcell' in styles and styles['headcell']) for x in headers]]+[[Paragraph(e(x).replace('\n','<br/>'),styles['cell']) for x in row] for row in rows]
        t=Table(data,colWidths=[CW*x for x in widths] if widths else [CW/len(headers)]*len(headers),hAlign='LEFT')
        t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,0),NAVY),('VALIGN',(0,0),(-1,-1),'TOP'),('LEFTPADDING',(0,0),(-1,-1),7),('RIGHTPADDING',(0,0),(-1,-1),7),('TOPPADDING',(0,0),(-1,-1),6),('BOTTOMPADDING',(0,0),(-1,-1),6),('ROWBACKGROUNDS',(0,1),(-1,-1),[colors.white,PALE]),('LINEBELOW',(0,-1),(-1,-1),.6,colors.HexColor('#beced6'))]))
        _,h=t.wrap(CW,1000);self.check(h+11);t.drawOn(self.c,M,self.y-h);self.y-=h+11
    def image(self,name,caption,height=310,shot=False):
        p=A/('screenshots' if shot else 'figures')/(name+'.png');im=PILImage.open(p);iw,ih=im.size
        h=min(height,CW*ih/iw);w=h*iw/ih;self.check(h+15);self.c.drawImage(ImageReader(im),M+(CW-w)/2,self.y-h,w,h,mask='auto');self.y-=h+6
        self.fig+=1;self.figures.append(dict(figure=self.fig,page=self.page,path=str(p.relative_to(ROOT)),caption=caption))
        self.p(f'<b>Hình {self.fig}.</b> {e(caption)}','caption')
    def note(self,title,text):self.p(title,'h2');self.p(text)
    def gallery(self,title,section,items,intro='',height=240):
        self.new(title,section)
        if intro:self.p(intro)
        for name,cap,shot in items:self.image(name,cap,height,shot)
    def finish(self):
        self.c.save();return dict(pages=self.pages,figures=self.figures,sections=self.sections)

def author(r):
    r.new('MÔ HÌNH 3D ROBOT VI SAI','TỔNG QUAN',True)
    r.p('Từ bản STEP đến mô phỏng di chuyển','h2')
    r.p('Báo cáo kỹ thuật chi tiết | STEP - URDF - SDF - Gazebo - RViz2 - Nav2')
    r.image('cad_iso_front','Dựng hình trực tiếp từ hình học STEP. Màu phân biệt hình học do báo cáo gán, không phải chứng nhận vật liệu; bao gồm solid và các mặt không thuộc solid.',335)
    r.table(['Hình học nguồn','Cấu trúc ROS','Thực nghiệm mới'],[['542 solid + 5.287 mặt ngoài solid','8 link / 7 joint','7 phép thử × 3 lần = 21 lượt'],['168 bản ghi PRODUCT','3 mesh STL hiển thị','1 lượt Nav2 thành công trong kho']],[.35,.28,.37])
    r.p('<b>Mục đích:</b> cung cấp hồ sơ có thể kiểm tra và tái sử dụng cho nghiên cứu chuyển động. Các khác biệt giữa CAD, mô hình mô phỏng và thông số phần cứng được giữ nguyên, giải thích rõ, không che bằng hình minh họa.')
    r.p('Workspace: agv_nav2_research_ws_bao<br/>Nguồn chính: file_3D/Xe.step và package vacuum_robot_gazebo<br/>Ngày phân tích và ghi dữ liệu: 05/10/2026 (UTC+07)','small')

    r.new('Kết luận chính trước khi đọc chi tiết','TÓM TẮT',True)
    r.note('Mô hình mô phỏng đã chạy được','URDF hiển thị và TF hoạt động trong RViz2; SDF sinh robot, bánh xe và cảm biến trong Gazebo. Đã ghi đủ 21 lượt thử động học cơ bản và một lượt Nav2 đi dọc hành lang ngang của kho giao cắt. Năm kiểm thử hợp đồng robot_description đều đạt.')
    r.table(['Kết quả quan trọng','Giá trị / cách hiểu'],[
        ['Bao thân mô phỏng','440 × 340 mm; chiều cao mesh thân 160,00 mm'],
        ['Khoảng cách tâm lốp STEP','230,80 mm, suy từ hai solid lốp #27 và #34'],
        ['Khoảng cách bánh URDF/SDF','254,80 mm; rộng hơn STEP 24,00 mm, tương đương 10,40%'],
        ['Thông số quay riêng của Gazebo','283,40 mm: giá trị hiệu dụng của DiffDrive, không phải kích thước cơ khí'],
        ['Khối lượng mô phỏng','5,00 kg theo SDF; không phải kết quả cân robot'],
        ['Dừng từ 0,30 m/s','Đi thêm 100,78 ± 0,51 mm; thời gian đến ngưỡng dừng 0,672 ± 0,006 s (n = 3)'],
        ['Chạy cung (0,15 m/s; ±0,30 rad/s)','Sai lệch vị trí cuối giữa odom và ground truth khoảng 17,1 mm'],
        ['Minh họa Nav2 trong kho','Action thành công; sai lệch cuối GT so với goal khoảng 56,46 mm']],[.39,.61])
    r.note('Không được đánh đồng ba mức bằng chứng','(1) Hình học lấy từ tệp nguồn; (2) thông số khai báo trong cấu hình; (3) dữ liệu ghi từ mô phỏng. Báo cáo này chưa đo robot thật, chưa xác nhận tải trọng cho phép, độ bền, dòng điện thực, tuổi thọ pin hoặc độ an toàn vận hành.')
    r.p('<b>Ưu tiên trước khi áp dụng ngoài thực tế:</b> chọn bản cơ khí chuẩn, đo lại b và bán kính lăn dưới tải, sau đó hiệu chuẩn odometry. Không sao chép b hiệu dụng của Gazebo sang robot thật.')

    r.new('Mục lục và lộ trình đọc','HƯỚNG DẪN',True)
    sections=(r.prior or {}).get('sections',[])
    if sections:
        for entry in sections:
            if entry['page']<=3:continue
            r.p(f'<link href="#p{entry["page"]}" color="#087f8c">{e(entry["title"])}</link> <font color="#546777">... {entry["page"]}</font>','small')
    else:r.p('Mục lục được điền tự động ở lượt dàn trang thứ hai.')
    r.note('Cách đọc theo nhu cầu','Đọc lần đầu: nguồn dữ liệu → hình học → đối chiếu mô hình → hệ tọa độ → động học → thí nghiệm. Dùng cho bài báo: xem phương pháp đo, bảng tổng hợp và giới hạn kết luận. Dùng để lấy hình: tra phụ lục danh mục hình và figure_page_index.json.')
    r.p('Ký hiệu nguồn: [S1] STEP; [S2] URDF/STL; [S3] SDF; [S4] cấu hình robot/Nav2/bridge/EKF; [E1] 21 lượt mô phỏng; [E2] lượt minh họa kho; [D] tính toán suy ra. Hình chụp giữ giao diện ứng dụng; hình dựng CAD và biểu đồ được ghi nhãn riêng.','small')

    r.new('1. Nguồn dữ liệu và chuỗi mô hình','01 / NGUỒN',True)
    r.image('model_pipeline','Sơ đồ vai trò của các tệp và các thành phần đang dùng. Mũi tên mô tả luồng phụ thuộc; STEP không tự động được chuyển lại sang URDF trong phiên báo cáo này.',288)
    r.table(['Nguồn','Vai trò và giới hạn'],[
        ['[S1] file_3D/Xe.step','CAD AP214; tiêu đề tệp ghi xuất từ Autodesk Translation Framework ngày 15/07/2026. Không suy ra tệp này là phiên bản mới nhất chỉ từ thời điểm người dùng sao chép vào workspace.'],
        ['[S2] urdf/vacuum_robot.urdf + meshes/*.stl','Mô tả cây link/joint, visual, collision và hai bánh. Mesh dùng hệ số 0,001 từ mm sang m.'],
        ['[S3] models/vacuum_robot/model.sdf','Mô hình động lực học đang được spawn; chứa khối lượng thân, vật lý tiếp xúc, DiffDrive, LiDAR, IMU.'],
        ['[S4] config/*.yaml','Thông số Nav2, bridge, profile phần cứng và EKF. Giá trị null nghĩa là chưa cấu hình, không được tự điền từ tên linh kiện.']],[.38,.62])
    r.p('Các đường dẫn trên nằm trong src/vacuum_robot_gazebo, trừ STEP. Bản được Gazebo nạp trong install/ là symlink về chính tệp nguồn đã phân tích. Mã SHA-256 và commit được lưu để truy nguyên.','small')

    r.new('2. Cấu tạo và hình học STEP','02 / CAD',True)
    r.image('cad_iso_back','Phối cảnh phía đối diện của STEP. Vị trí các tấm sàn, hai cụm truyền động, trụ đỡ và cụm quét phía trên được giữ theo tệp CAD.',310)
    r.table(['Đại lượng','Kết quả đọc hình học'],[
        ['Số solid / mặt không thuộc solid','542 / 5.287. Mặt rời được dựng cùng solid; không được bỏ qua khi đánh giá bao hình học.'],
        ['Bản ghi PRODUCT','168. Đây không phải bảng kê số lượng vật tư đã xác minh.'],
        ['Bao toàn STEP theo trục gốc X/Y/Z','340,000 × 166,374 × 440,000 mm (hộp bao OCCT của toàn shape).'],
        ['Tổng thể tích các solid','3.551.284,44 mm³. Cộng thể tích từng solid; không trừ giao nhau và không bao gồm thể tích của mặt hở.'],
        ['Lưới hóa để dựng hình','Độ lệch tuyến tính 0,35 mm; góc 0,25 rad. Số tam giác riêng các solid: 1.666.785.']],[.4,.6])
    r.p('Không đổi tổng thể tích này thành khối lượng: cần vật liệu/mật độ đúng cho từng chi tiết, trạng thái lắp ghép và kiểm tra giao nhau. Khối lượng mô phỏng được lấy độc lập từ SDF.','small')
    r.gallery('2.1. Hình chiếu bằng và mặt dưới','02 / CAD',[
        ('cad_top','STEP nhìn từ trên, sau khi quy trục dài theo x, ngang theo y, cao theo z. Hình chiếu song song, không có phối cảnh.',False),
        ('cad_bottom','STEP nhìn từ dưới để thấy bánh xe, các vị trí đỡ và mặt đáy. Đây là phép đổi góc nhìn, không đảo cấu trúc lắp ghép.',False)],height=245)
    r.gallery('2.2. Hai mặt đầu của robot','02 / CAD',[
        ('cad_front','Mặt đầu theo +x của hệ trục quy ước dùng để đối chiếu với URDF. Không coi nhãn “trước” là xác nhận hướng lắp phần cứng.',False),
        ('cad_rear','Mặt đầu đối diện. Hình giúp phân biệt bao ngoài thân với cụm cảm biến nằm phía trên.',False)],height=240)
    r.gallery('2.3. Hai mặt bên và vị trí bánh','02 / CAD',[
        ('cad_left','Mặt bên +y của STEP. Quan sát lốp, cụm truyền động và khe khoét của các tấm sàn.',False),
        ('cad_right','Mặt bên -y. Các mặt hở trang trí/chi tiết lốp vẫn được dựng; màu là màu kỹ thuật do báo cáo gán.',False)],height=245)
    r.gallery('2.4. Kết cấu khung và bố trí bên trong','02 / CAD',[
        ('cad_structure','Bốn solid kết cấu lớn #20-23: hai tấm sàn và hai vùng vách. Nhóm được chọn theo hình học, không phải mã linh kiện của BOM.',False),
        ('cad_open_view','Minh họa bóc phần phía trên bằng cách ẩn tam giác có trọng tâm cao hơn mặt phẳng cắt. Không phải mặt cắt BRep kín và không dùng để đo chiều dày tại mép cắt.',False)],height=235)
    r.gallery('2.5. Hai cụm động cơ và bánh xe','02 / CAD',[
        ('cad_drive_a','Cụm bên A: giá đỡ, thân motor, hộp số/trục và phần solid bánh. Danh sách solid được lưu trong cad_audit.json.',False),
        ('cad_drive_b','Cụm bên B, dựng với cùng góc nhìn. Phân nhóm bằng vị trí và hình học; không suy ra tỷ số truyền hay điện áp chỉ từ hình CAD.',False)],height=240)
    r.gallery('2.6. Điện tử và cụm quét','02 / CAD',[
        ('cad_electronics','Các solid nhỏ trong vùng điện tử được chọn bằng hộp không gian, giữ nguyên vị trí. Không phải sơ đồ đấu nối điện.',False),
        ('cad_lidar','Nhóm hình học cảm biến phía trên, solid #527-533. Mặt phát tia thực tế trong mô phỏng được định nghĩa bằng pose của sensor SDF, không bằng màu hay bề mặt này.',False)],height=235)
    r.new('2.7. Tách lớp để hiểu lắp ghép','02 / CAD')
    r.image('cad_exploded','Hình tách giãn các khối CAD để giải thích cấu trúc; vị trí đã được dịch khỏi lắp ghép gốc. Không dùng hình này để đo khoảng cách lắp ráp hoặc quy trình tháo lắp.',345)
    r.note('Tên linh kiện có trong metadata STEP','Các bản ghi PRODUCT có GA25-370_GEAR_MOTOR, 85MM WHEEL, BTS7960 43A HIGH POWER H BRIDGE MODULE, XL4016, raspberry_pi_5, pin3khay, manhinh và các chi tiết khác. Tên mô hình không xác nhận thiết bị thực tế đã lắp, chất lượng linh kiện hoặc thông số định mức.')
    r.p('Profile hiện để motor_driver và controller là null. Vì vậy, việc thấy mô hình BTS7960/Raspberry Pi trong STEP chỉ chứng minh dữ liệu CAD chứa tên/hình học tương ứng; chưa chứng minh driver ROS và đường điều khiển phần cứng đã hoàn chỉnh.')

    r.new('3. STEP và URDF chưa trùng hoàn toàn','03 / ĐỐI CHIẾU',True)
    r.image('track_difference','Đối chiếu tâm lốp trong STEP với vị trí joint/collision bánh của URDF/SDF. Bao thân 440 × 340 mm được giữ làm mốc chung.',245)
    r.table(['Đại lượng','STEP','URDF/SDF'],[
        ['Chiều dài × rộng','440 × 340 mm','440 × 340 mm'],
        ['Khoảng cách tâm phần lốp','230,80 mm','254,80 mm'],
        ['Bề rộng solid lốp / tread collision','30,00 mm','30,00 mm'],
        ['Đường kính phần solid lốp','≈85,001 mm','Collision danh định 85,00 mm'],
        ['Bao chi tiết mặt rời tại bánh','≈88,744 mm theo phương đứng của lưới hóa','Mesh bánh ≈84,995 mm; không có cùng bao chi tiết mặt rời'],
        ['Bao cao toàn hình học','166,374 mm (OCCT, gồm mặt hở)','160,000 mm (mesh thân)']],[.43,.29,.28])
    r.p('<b>Hệ quả:</b> khác biệt 24 mm của b không biến mất khi đổi trục hoặc tịnh tiến. Nếu dùng b = 254,8 mm để tính yaw từ vận tốc hai bánh của một xe thật có b = 230,8 mm, yaw ước lượng sẽ thấp hơn khoảng 9,42% trong mô hình lăn lý tưởng. Điều này chưa bao gồm trượt, biến dạng lốp và sai số bán kính.')

    r.new('3.1. Quy trục và đối chiếu bề mặt','03 / ĐỐI CHIẾU')
    r.image('cad_urdf_overlay','Chồng hình học STEP (cam) và URDF visual (xanh). Quy trục x = Z_STEP, y = -X_STEP, z = -Y_STEP - 20 mm để neo theo trục bánh; không ép đổi kích thước.',270)
    r.note('Phương pháp không che mất sai khác','Thử 24 phép hoán vị trục có định hướng thuận, không fit hệ số tỷ lệ. Phép tìm ban đầu dùng tâm hộp bao để so khớp; hình này dùng mốc trục bánh -20 mm theo Y gốc của STEP. Do thân gần đối xứng, dấu trục được chọn để tiện đối chiếu, không phải xác nhận hướng mũi xe vật lý.')
    r.p('Các sai khác gồm vị trí lốp theo phương ngang, bao chi tiết bề mặt của lốp và một số chi tiết bên trong/thân trên. Hình học mặt hở trong STEP phải được giữ khi kiểm tra envelope; 542 solid đơn thuần chỉ có bao cao khoảng 164,501 mm.')
    r.p('registration.json còn lưu kiểm tra khoảng cách tới đám mây điểm lấy mẫu trên bề mặt. Chỉ số này chịu ảnh hưởng mật độ mẫu, mặt trong và khác biệt cấu trúc; không sử dụng như dung sai chế tạo, Hausdorff chính xác hoặc chứng minh hai bản trùng nhau.','small')
    r.gallery('3.2. Bánh xe: solid, mặt rời và mesh','03 / ĐỐI CHIẾU',[
        ('cad_tread_surface','Hai solid lốp và các mặt rời quanh bánh trong STEP. Bao gân/chi tiết ngoài không đồng nghĩa bán kính lăn hữu hiệu khi chịu tải.',False),
        ('mesh_1','Mesh bánh trái đang dùng trong URDF, đã áp dụng scale và transform của visual. Bao phần mesh theo trục bánh dài 42,4 mm; tread collision chỉ rộng 30 mm.',False)],height=240)

    r.new('4. URDF, link và hệ tọa độ','04 / MÔ TẢ ROBOT',True)
    r.image('tf_tree','Cây 8 link và 7 joint của URDF. Các tọa độ ghi tương đối với base_link; hai wheel joint là continuous, năm joint còn lại fixed.',280)
    r.table(['Frame / link','Vị trí tương đối (mm)','Ý nghĩa'],[
        ['base_link','0; 0; 0','Mốc tại tâm trục truyền động'],['base_footprint','0; 0; -42,50','Hình chiếu xuống mặt sàn danh định'],
        ['laser','0; 0; +108,92','Khung bản tin LiDAR'],['imu_link','0; 0; -12,80','Khung IMU'],
        ['left_motor / right_motor','0; ±93,40; 0','Visual mô tả motor'],['left_wheel / right_wheel','0; ±127,40; 0','Khớp quay quanh trục +y']],[.34,.29,.37])
    r.p('Quy ước dùng trong báo cáo: +x hướng tiến, +y bên trái, +z lên; yaw dương quay trái. SDF đặt base_link tại z = 42,5 mm so với model; do đó link laser cao danh định 151,42 mm so với sàn.','small')
    r.gallery('4.1. Bao ngoài của mesh mô phỏng','04 / MÔ TẢ ROBOT',[
        ('urdf_iso','Render ba mesh STL với đúng transform URDF. Màu kỹ thuật của báo cáo, không phải ảnh Gazebo/RViz2.',False),
        ('urdf_top','Nhìn từ trên của các mesh mô phỏng. Vị trí bánh thuộc URDF hiện hành, không dùng trực tiếp vị trí bánh trong STEP.',False)],height=240)
    r.gallery('4.2. Mặt bên và đáy của URDF visual','04 / MÔ TẢ ROBOT',[
        ('urdf_side','Mặt bên: dễ quan sát tương quan giữa trục bánh, mặt đáy và đỉnh cụm LiDAR.',False),
        ('urdf_bottom','Mặt dưới: visual chỉ là hình học hiển thị. Các tiếp xúc trong vật lý do collision primitives quyết định.',False)],height=240)
    r.new('4.3. Mesh và các phép biến đổi','04 / MÔ TẢ ROBOT')
    r.image('urdf_exploded','Tách giãn ba mesh để phân biệt thân và hai bánh. Đây là hình giải thích; không phải trạng thái thực trong Gazebo.',285)
    r.table(['Mesh','Tam giác','Kín nước','Tịnh tiến tổng trong base_link (m)'],[
        ['base_link.stl','67.490','Không','-0,035315; +0,200943; -0,211698'],
        ['left_wheel_link_1.stl','21.050','Có','-0,0353034; +0,2009430; -0,2091883'],
        ['right_wheel_link_1.stl','21.050','Có','-0,0353129; +0,2009430; -0,2091955']],[.32,.13,.14,.41])
    r.p('Cả ba mesh dùng scale = 0,001. Tịnh tiến tổng của bánh đã cộng origin của wheel joint; không cộng lần nữa khi dựng hình trong base_link. rpy của các visual mesh bằng 0. “Kín nước” do kiểm tra topo mesh bằng trimesh, không phải phép thử kín cơ khí.')
    r.p('Mesh thân không kín vẫn có thể hiển thị đúng. Vì collision được mô tả riêng bằng primitive, không cần suy ra volume/inertia từ mesh thân này. Chỉ dùng thể tích khi đã xác minh mesh và mật độ phù hợp.')

    r.new('5. Collision, khối lượng và tiếp xúc','05 / VẬT LÝ',True)
    r.image('rviz_collision','Ảnh chụp RViz2 thật: tắt Visual, bật Collision. Hình vuông/khối đầy là mô hình va chạm, không phải thiếu chi tiết CAD.',250,True)
    r.table(['Collision','Kích thước (m)','Tâm trong base_link (m)'],[
        ['lower_center','Box 0,44 × 0,20 × 0,10','0; 0; 0,0675'],['front_cross / rear_cross','Box 0,12 × 0,34 × 0,10','±0,16; 0; 0,0675'],
        ['upper_deck','Box 0,44 × 0,34 × 0,04','0; 0; 0,0975'],['4 ball support','Sphere r = 0,006','x = ±0,15; y = ±0,09; z = -0,0365'],
        ['2 bánh','Cylinder r = 0,0425; dài 0,030','0; ±0,1274; 0; trục cylinder xoay sang y']],[.32,.35,.33])
    r.p('Các box thân giao nhau có chủ ý, tạo bao va chạm bảo thủ, không phải các thể tích để cộng khối lượng. Đáy box lower_center cao 60 mm so với sàn; vì vậy mô hình collision không mô tả mọi bề mặt thấp của CAD. Không dùng nó để kết luận khả năng vượt gờ/vật thấp của robot thật.')
    r.new('5.1. Khối lượng, COM và tensor quán tính','05 / VẬT LÝ')
    r.image('mass_inertia','Khối lượng khai báo trong SDF và các mômen quán tính chính của toàn mô hình, quy về COM bằng định lý trục song song [S3, D].',230)
    r.table(['Thành phần','m (kg)','COM theo mặt sàn (mm)'],[
        ['Thân và phần cứng đã gộp','4,60','-6,074; -0,536; 58,523'],['Bánh trái','0,20','-0,010; 127,246; 42,487'],['Bánh phải','0,20','-0,000; -127,246; 42,501'],
        ['Toàn mô hình','5,00','-5,588; -0,493; 57,241']],[.38,.15,.47])
    r.p('<b>Tensor quán tính tổng tại COM (kg·m²), theo trục base_link:</b><br/>[ 0,04952908 &nbsp; 0,00032669 &nbsp; -0,00088495 ]<br/>[ 0,00032669 &nbsp; 0,10812457 &nbsp; 0,00011532 ]<br/>[ -0,00088495 &nbsp; 0,00011532 &nbsp; 0,14403043 ]')
    r.p('Các trị riêng dương: 0,04951896; 0,10812606; 0,14403906 kg·m², đồng thời thỏa bất đẳng thức tam giác mômen quán tính. Đây là kiểm tra tính khả dĩ toán học; không xác nhận quán tính đo thực tế.')
    r.p('URDF cố ý không đặt inertia cho root base_link; SDF mới chứa 4,6 kg của thân. Hai motor visual không có mass/collision riêng để tránh cộng trùng phần cứng đã gộp. Nếu dùng lại URDF cho một backend động lực học khác, cần xem lại cách mô tả inertial.','small')
    r.new('5.2. Điểm đỡ, ma sát và tải thêm','05 / VẬT LÝ')
    r.image('support_payload','Bên trái: đa giác nối các điểm tiếp xúc danh định. Bên phải: ví dụ tính COM khi thêm một tải giả định, chỉ minh họa công thức; không phải kết quả thử tải.',245)
    r.table(['Thông số SDF','Giá trị','Giới hạn diễn giải'],[
        ['Mu bánh / mu2','1,2 / 0,05','Có tính hướng trong cấu hình; không đồng nghĩa hệ số đã đo của lốp'],['Slip1 / slip2 bánh','0 / 0,02','Tham số tiếp xúc, phụ thuộc backend vật lý'],['Mu thân / ball','0,02 / 0,005','Mô hình hóa mặt trượt và đỡ, không phải vật liệu được nhận dạng'],['Damping mỗi joint','0,03','Không có mô hình điện-động cơ đã nhận dạng'],['Self collision','false','Không kiểm tra va chạm nội bộ giữa các link bằng cấu hình này']],[.34,.19,.47])
    r.p('GUI phiên chạy cho thấy backend gz-physics-dartsim-plugin, collision detector ode. Không suy ra toàn bộ backend chỉ từ thuộc tính type="ode" của world. Cần kiểm tra ảnh hưởng thật của các tham số friction/slip nếu làm nghiên cứu ma sát.')
    r.p('Với tải thêm mₚ tại cao độ zₚ: z_COM = (5 × 0,0572407 + mₚzₚ)/(5 + mₚ). Chưa có dữ liệu độ bền khung, lực nén điểm đỡ, dòng motor hoặc thử lật; báo cáo không đưa ra tải trọng cho phép.')

    r.new('6. Cảm biến và giao tiếp ROS 2','06 / CẢM BIẾN',True)
    r.image('geometry_layout','Sơ đồ kích thước, điểm đỡ và cao độ cảm biến danh định từ URDF/SDF. Các khối thân bên phải là sơ đồ đơn giản, không phải mặt cắt CAD.',270)
    r.table(['Cảm biến','Cấu hình mô phỏng','Frame / topic'],[
        ['LiDAR gpu_lidar','5,5 Hz; 1.440 tia; 360°; 0,15-12 m; độ phân giải range 0,01 m; nhiễu Gaussian σ = 0,01 m','laser /scan'],
        ['IMU','100 Hz; gyro σ = 0,002 rad/s; accel σ = 0,03 m/s²; mô phỏng theo sensor SDF','imu_link /imu/data'],
        ['Odometry bánh','Khai báo 30 Hz; DiffDrive sử dụng b hiệu dụng 0,2834 m','odom → base_link; /odom'],
        ['Ground truth','OdometryPublisher; dimensions = 2; khai báo 30 Hz; tách khỏi odometry bánh','world /ground_truth/odom']],[.23,.54,.23])
    r.p('Laser không phải camera 3D và không quét toàn chiều cao robot. Vật nằm thấp hơn hoặc cao hơn mặt quét có thể không tạo dữ liệu LiDAR phù hợp, dù vẫn liên quan tới va chạm cơ khí. Chưa có phép thử đánh giá vùng mù trên phần cứng.','small')
    r.new('6.1. Tần số thực nhận và đồng bộ','06 / CẢM BIẾN')
    r.image('sample_rates','Thống kê timestamp ghi ở E1. Tần số trung bình khác tần số cấu hình do bước vật lý 3 ms, lịch xuất bản và quá trình reset/ghi nhận.',260)
    r.table(['Luồng','Số mẫu','1/mean(Δt) Hz','Trung vị Δt (ms)'],[[{'truth':'Ground truth','odom':'Odometry','imu':'IMU','joints':'JointState','scan':'LaserScan','commands':'Lệnh ghi'}[k],str(RATES[k]['samples']),num(1/RATES[k]['mean_period_s'],2),num(1000/RATES[k]['median_hz'],2)] for k in ['truth','odom','imu','joints','scan','commands']],[.28,.20,.26,.26])
    r.p('Ví dụ IMU: trung vị khoảng mẫu là 9 ms, nhưng có cả khoảng 12 ms; lấy 1/9 ms rồi tuyên bố IMU chạy 111 Hz sẽ gây hiểu sai. Báo cáo dùng thêm tần số theo khoảng mẫu trung bình, lưu cả hai cách tính để kiểm tra.')
    r.p('Tất cả chỉ số thời gian thí nghiệm dùng thời gian mô phỏng và header timestamp. Không thay bằng thời gian chờ trên máy tính. Các pha teleport có nhãn reset, bị loại khỏi tính kết quả từng lượt nhưng vẫn nằm trong thống kê tốc độ ghi toàn phiên.','small')

    r.new('7. Động học và giới hạn chuyển động','07 / ĐỘNG HỌC',True)
    r.p('Với r = 0,0425 m; b là khoảng cách bánh phù hợp với mô hình đang xét:<br/><b>v_L = v - bω/2; v_R = v + bω/2</b><br/><b>q̇_L = v_L/r; q̇_R = v_R/r</b><br/><b>v = r(q̇_R + q̇_L)/2; ω = r(q̇_R - q̇_L)/b</b>')
    r.image('velocity_envelope','Trái: miền ghép các giới hạn v, ω và tốc độ bánh 0,36 m/s. Phải: bán kính từ các ràng buộc khác nhau, không phải một “bán kính quay robot” duy nhất.',250)
    r.table(['Ví dụ lệnh','q̇L; q̇R với b = 0,2548 m','Với b_eff = 0,2834 m'],[
        ['v = 0,20; ω = 0','4,706; 4,706 rad/s','4,706; 4,706 rad/s'],['v = 0; ω = 0,50','-1,499; 1,499 rad/s','-1,667; 1,667 rad/s'],['v = 0,15; ω = 0,30','2,630; 4,429 rad/s','2,529; 4,530 rad/s'],['v = 0,30; ω = 0,80','4,661; 9,457 rad/s','4,392; 9,726 rad/s']],[.29,.36,.35])
    r.p('Ở góc v = 0,30; ω = 0,80, bánh ngoài theo hình học đạt 0,40192 m/s &gt; 0,36 m/s của PSTMO. Vì vậy hai giới hạn v/ω độc lập không bảo đảm giới hạn bánh; cần kiểm tra miền ghép. 0,36 m/s là cấu hình PSTMO, không phải khẳng định plugin Gazebo tự áp ràng buộc đó.','small')
    r.new('7.1. Footprint và không gian quay','07 / ĐỘNG HỌC')
    r.image('swept_footprint','Minh họa hình học từ footprint 440 × 340 mm: quét khi xoay tại chỗ và khi đi cung R = 0,50 m. Không phải quỹ đạo thử nghiệm mới.',275)
    r.table(['Ứng dụng','Kết quả suy ra / lưu ý'],[
        ['Đi thẳng giữa hai vách song song','Bề rộng hình học tối thiểu 340 mm khi đã thẳng hướng; phải cộng sai số định vị và khoảng hở yêu cầu.'],
        ['Quay tự do tại tâm trục','Bán kính bao góc footprint ≈278,03 mm; đường kính quét ≈556,06 mm, chưa cộng biên an toàn.'],
        ['Inflation của costmap','0,45 m trong YAML. Đây là vùng chi phí, không được diễn giải trực tiếp thành khoảng hở luôn được bảo đảm.'],
        ['SmacHybrid','Minimum turning radius 0,35 m là ràng buộc planner; robot vi sai vẫn có mô hình quay tại chỗ.'],
        ['Kiểm tra góc giao cắt','Cần kiểm tra toàn footprint đã xoay theo yaw, không chỉ khoảng cách của tâm robot tới kệ.']],[.36,.64])
    r.p('Footprint trong Nav2 là bao chữ nhật. Bao này cố ý không đi theo từng khe khoét CAD; nó đơn giản và bảo thủ hơn silhouette ở một số vị trí, nhưng chưa mô tả rủi ro theo chiều cao.','small')
    r.new('7.2. Động cơ, truyền động và nguồn','07 / ĐỘNG HỌC')
    r.table(['Nhóm dữ liệu từ profile','Giá trị khai báo','Trạng thái bằng chứng'],[
        ['Motor','2 × GA25, 12 V, giảm tốc 45:1','Theo profile, chưa đo thực'],['Tốc độ không tải / định mức','130 / 100 rpm','Lý thuyết theo thông số đã nhập'],['Vận tốc tuyến tính tương ứng','0,5786 / 0,4451 m/s','2πr × rpm / 60, chưa có tải và trượt'],['Torque định mức / stall','0,0980665 / 0,3530394 N·m','Không coi stall là torque liên tục cho phép'],['Giới hạn joint','13,61357 rad/s; effort 0,3530394 N·m','Khai báo SDF/URDF, không chứng minh mô hình điện áp-dòng điện'],['Giới hạn chuyển động mô phỏng','|v| ≤ 0,30 m/s; |ω| ≤ 0,80 rad/s','DiffDrive thực thi giới hạn vận tốc/gia tốc'],['Pin','4S4P, 16 cell; 10,4 Ah; 153,92 Wh danh định','Giá trị từ profile với giả định 3,7 V/cell'],['BMS, cầu chì, bộ hạ áp, driver','Chưa có định mức đầy đủ; nhiều trường null','Không có kết luận an toàn điện hay thời lượng thực']],[.29,.36,.35])
    r.note('Giới hạn cần nhớ khi mô phỏng','Transmissions trong URDF dùng SimpleTransmission và hardware_interface/VelocityJointInterface. Sự tồn tại của các thẻ này không chứng minh ros2_control/hardware interface đang điều khiển motor thật; phiên mô phỏng điều khiển bằng plugin DiffDrive.')
    r.p('Không tính thời lượng pin chỉ bằng chia Wh cho công suất motor danh định: còn máy tính, cảm biến, converter, hiệu suất, duty cycle và đặc tính tải. Báo cáo không đưa ra sơ đồ đấu nguồn hoặc xác nhận tương thích điện áp thiết bị.')
    r.new('7.3. Chuyển sang robot thật: dữ liệu còn thiếu','07 / ĐỘNG HỌC')
    r.table(['Hạng mục','Cấu hình hiện có','Dữ liệu phải xác nhận'],[
        ['Encoder','location/PPR/decoding/ticks_per_rev đều null','Vị trí đo trước/sau hộp số; số xung và giải mã; dấu từng bánh'],
        ['Odometry phần cứng','EKF dự kiến nhận /wheel/odometry','r_L, r_R, b thực và hệ số hiệu chuẩn riêng của robot thật'],
        ['IMU','BNO055; EKF chỉ fuse gyro yaw rate','Mounting/extrinsic, bias, covariance, nhiễu từ, driver và transport'],
        ['LiDAR','Tên RPLIDAR A1M8; serial 115200 trong profile','Thiết bị thực, driver, frame và pose đã đo; độ trễ scan'],
        ['TF odom → base_link','Gazebo DiffDrive sở hữu trong mô phỏng','Khi dùng EKF thật: chỉ một node sở hữu transform, tránh phát trùng'],
        ['Timeout','Profile command_timeout 0,25 s; smoother 0,15 s','Xác nhận watchdog được triển khai ở phần cứng; không suy ra từ YAML'],
        ['Khối lượng / tải','5 kg được khai báo trong SDF','Cân robot, tải thêm, COM và quán tính phù hợp phiên bản cơ khí']],[.25,.37,.38])
    r.note('Thông tin đủ để thiết kế phép đo, chưa đủ để công bố phần cứng','Báo cáo cung cấp điểm xuất phát để xây dựng thí nghiệm. Trước khi viết “validated on a physical robot”, cần dữ liệu robot thật độc lập, ground truth đo ngoài, quy trình an toàn và điều kiện môi trường được ghi lại.')

    r.new('8. Ảnh Gazebo và RViz2 thực tế','08 / MINH CHỨNG',True)
    r.image('gz_iso_front','Ảnh chụp cửa sổ Gazebo Sim 8.15.0: robot ở vùng trống của open_arena. Không phải ảnh dựng CAD hoặc hình tạo sinh.',285,True)
    r.image('gz_iso_rear','Gazebo: góc đối diện để quan sát bánh còn lại và kết cấu thân. Camera/giờ chụp được lưu trong capture_manifest.jsonl.',285,True)
    r.gallery('8.1. Gazebo: mặt đầu và mặt bên','08 / MINH CHỨNG',[
        ('gz_front','Gazebo: nhìn gần ngang trục dọc. Robot dùng nguyên model.sdf của workspace.',True),
        ('gz_side','Gazebo: mặt bên, kiểm tra cao độ bánh và cảm biến. Màu sáng lấy trực tiếp từ visual/material của mô hình.',True)],height=280)
    r.gallery('8.2. Gazebo: mặt trên và bánh xe','08 / MINH CHỨNG',[
        ('gz_top','Gazebo: nhìn từ trên. Hình dùng để nhận diện bao thân và phân bố bộ phận, không dùng đo pixel thay thế nguồn CAD.',True),
        ('gz_wheel_detail','Gazebo: góc cận cảnh bánh và vùng khoét thân. Collision và contact lực vẫn là mô hình đơn giản hóa ở mục 5.',True)],height=280)
    r.gallery('8.3. RViz2: mô hình hiển thị và góc sau','08 / MINH CHỨNG',[
        ('rviz_iso','RViz2: RobotModel Status OK, Fixed Frame = base_link, nhận /robot_description. Giao diện được giữ để xác nhận nguồn ảnh.',True),
        ('rviz_rear','RViz2: góc nhìn đối diện với cùng mô hình URDF và joint state từ phiên Gazebo.',True)],height=280)
    r.gallery('8.4. RViz2: mặt trên và mặt bên','08 / MINH CHỨNG',[
        ('rviz_top','RViz2: hình chiếu gần thẳng đứng để đối chiếu vị trí hai bánh.',True),
        ('rviz_side','RViz2: nhìn cạnh, quan sát hệ cao độ của mô hình. Các grid 0,1 m chỉ là lưới tham chiếu hiển thị.',True)],height=280)
    r.gallery('8.5. RViz2: TF và điểm quét LiDAR','08 / MINH CHỨNG',[
        ('rviz_tf','RViz2 hiển thị TF của base_link, laser và hai bánh, làm mờ visual để thấy các gốc trục. Cây đầy đủ gồm cả motor và IMU được trình bày ở mục 4.',True),
        ('rviz_scan','RViz2 hiển thị /scan trong open_arena. Điểm quét là dữ liệu sensor đang chạy, không phải chấm vẽ tay.',True)],height=280)

    r.new('9. Tích hợp trong kho có lối giao cắt','09 / NAV2',True)
    r.image('gz_warehouse_overview','Gazebo: toàn cảnh warehouse_cross_aisles từ world SDF hiện có. Cùng mô hình robot được spawn ở đầu hành lang ngang.',285,True)
    r.image('gz_warehouse_top','Gazebo: nhìn từ trên để thấy các dãy kệ và hành lang giao cắt. Đây là cùng map kho của nghiên cứu trước, không phải map thiết kế mới.',285,True)
    r.gallery('9.1. Bản đồ, costmap và footprint','09 / NAV2',[
        ('rviz_warehouse_overview','RViz2: map và global costmap trong warehouse_cross_aisles. Các lớp được nạp từ Nav2 của phiên thực tế.',True),
        ('rviz_warehouse_local','RViz2: góc cận robot và polygon footprint. Bao chữ nhật màu vàng giúp liên hệ hình học cơ khí với costmap 2D.',True)],height=280)
    r.gallery('9.2. Robot chạy và đến đích','09 / NAV2',[
        ('rviz_warehouse_navigation','Ảnh RViz2 chụp trong lúc action NavigateToPose đang thực thi. Planner được chọn rõ là ThetaStar qua /planner_selector.',True),
        ('rviz_warehouse_arrival','Ảnh RViz2 sau khi action trả về SUCCEEDED (status = 4). Thành công action không thay thế kiểm tra ground truth.',True)],height=280)
    r.new('9.3. Số liệu lượt minh họa trong kho','09 / NAV2')
    r.image('warehouse_trajectory','Quỹ đạo ground truth mới ghi trong kho; các khối kệ được dựng lại từ collision box của world. Hình không phải một trong 21 lượt thử động học ở open_arena.',300)
    r.table(['Chỉ số','Kết quả'],[['Start / goal','(-5; 0; 0) → (5; 0; 0), đơn vị m và rad'],['Planner / action','ThetaStar / NavigateToPose / SUCCEEDED'],['Chiều dài từ ground truth',num(WM['path_length_m'],4)+' m'],['Sai lệch vị trí cuối so với goal',num(WM['goal_error_m']*1000,2)+' mm'],['Yaw cuối theo ground truth',num(WM['end_yaw_deg'],3)+'°'],['Cửa sổ ghi / số mẫu',num(WM['recorded_duration_s'],3)+' s / '+str(WM['samples'])+' mẫu']],[.47,.53])
    r.p('Sai lệch GT-goal ở đây dựa trên quy ước map/world trùng hệ tọa độ của kho sinh từ SDF. Cửa sổ ghi gồm thời gian trước action và quan sát sau action, không được gọi là thời gian action thuần. Chỉ một lượt minh họa: chưa có ý nghĩa so sánh thuật toán hay đánh giá xác suất thành công.','small')
    r.p('Lần thử khởi tạo đầu tiên đã abort vì BT mặc định yêu cầu GridBased, trong khi package chỉ có các planner đã đặt tên. Đã chọn ThetaStar bằng topic rồi chạy lại, không sửa cấu hình nguồn. Hồ sơ lần abort được giữ tại warehouse_demo_attempt1.json.','small')

    r.new('10. Phương pháp thử chuyển động','10 / THỰC NGHIỆM',True)
    r.p('Mục tiêu là kiểm tra đáp ứng của mô hình cơ học và tín hiệu ROS, không xếp hạng planner/smoother. Các lệnh E1 gửi trực tiếp /cmd_vel vào DiffDrive; Nav2 tắt để không trộn hành vi điều khiển đường đi với đáp ứng cơ bản.')
    r.table(['Ca thử','v (m/s)','ω (rad/s)','Giữ lệnh','Lặp'],[['Tiến thẳng','+0,20','0','5 s','3'],['Lùi thẳng','-0,20','0','5 s','3'],['Quay trái tại chỗ','0','+0,50','5 s','3'],['Quay phải tại chỗ','0','-0,50','5 s','3'],['Cung trái','+0,15','+0,30','6 s','3'],['Cung phải','+0,15','-0,30','6 s','3'],['Tiến rồi dừng','+0,30','0','4 s','3']],[.34,.16,.18,.18,.14])
    r.note('Quy trình cho mỗi lượt','Phát zero 1 s; đặt lại model tại (-3; -2; 0,002 m), yaw = 0; chờ ổn định 1,5 s; ghi baseline 1 s; phát lệnh ở khoảng 20 Hz theo thời gian mô phỏng; sau khoảng giữ lệnh phát zero và ghi thêm 3 s. Reset chỉ thay pose, không giả định joint positions hoặc odometry về zero.')
    r.p('Môi trường: open_arena, vùng chọn đã kiểm tra tránh các khối vật cản. ROS_DOMAIN_ID = 183 và GZ_PARTITION = robot3d_report tách phiên thử khỏi hệ khác. Phiên kho dùng domain 184 và partition riêng. Bước vật lý của world là 0,003 s.')
    r.p('Bản ghi giữ riêng từng luồng: truth.csv, odom.csv, joints.csv, imu.csv, scan.csv, commands.csv. Scan CSV lưu thống kê; một bản tin range đầy đủ nằm trong scan_snapshot.json. Không có dữ liệu dòng motor hoặc lực contact vì các luồng đó chưa được ghi.','small')
    r.new('10.1. Định nghĩa chỉ số và cách đọc đồ thị','10 / THỰC NGHIỆM')
    r.image('paths_scan','Trái: các quỹ đạo mẫu ở hệ tọa độ cục bộ đầu lượt. Phải: một LaserScan thực ghi trong phiên, quy về frame laser; không phải bản đồ toàn cục.',235)
    r.table(['Chỉ số','Định nghĩa sử dụng'],[
        ['Quãng đường L','Tổng chuẩn Euclid của chênh lệch các vị trí ground truth liên tiếp, gồm tăng tốc, giữ lệnh và dừng.'],
        ['Vận tốc thực','Đạo hàm số x,y và yaw ground truth theo timestamp; v chiếu lên hướng thân, ω từ yaw đã unwrap.'],
        ['Vùng ổn định','Từ t_start + 1,5 s đến t_stop - 0,3 s; không dùng cả giai đoạn quá độ để tính trung bình ổn định.'],
        ['Sai số odometry','Odom và GT được đưa riêng về pose đầu lượt; nội suy odom theo timestamp GT, không so hai vị trí toàn cục sau teleport.'],
        ['Quãng dừng','Tích phân rời rạc quãng đường sau timestamp bắt đầu phát zero; có nội suy vị trí tại mốc dừng.'],
        ['Thời gian dừng','Mẫu đầu sau lệnh zero thỏa |v| < 0,01 m/s và |ω| < 0,02 rad/s; không yêu cầu duy trì ngưỡng nhiều mẫu.'],
        ['Trung bình ± SD','n = 3; SD mẫu (ddof = 1). Nhiều đường trùng nhau do mô phỏng gần tất định, không chứng minh độ chính xác thực.']],[.28,.72])
    r.p('Dấu v và ω giữ nguyên: lùi và quay phải mang dấu âm. Sai lệch vị trí odometry là độ lớn; sai lệch yaw dùng GT - odom, có dấu.','small')

    insights={
      'forward':('Đáp ứng đi thẳng','Vận tốc ổn định đạt gần 0,2000 m/s. Quãng đường tổng khoảng 0,9875 m không bằng đúng 0,20 × 5: mất diện tích vận tốc ở pha tăng tốc và bù một phần ở pha dừng. Sai lệch odom-GT rất nhỏ trong mô phỏng này không có nghĩa encoder thực sẽ chính xác dưới micromet.'),
      'reverse':('Lùi không hoàn toàn đối xứng với tiến','Quãng đường trung bình khoảng 1,0127 m, lớn hơn thử tiến. Giới hạn gia tốc có dấu của DiffDrive là [-0,45; +0,35] m/s²: khi lùi, pha tăng độ lớn vận tốc dùng hướng âm còn khi hãm về zero dùng hướng dương. Đây là giải thích từ cấu hình, không phải bằng chứng ma sát thực bất đối xứng.'),
      'pivot_ccw':('Quay tại chỗ và hiệu chỉnh b','Tốc độ quay ổn định khoảng +0,49755 rad/s với lệnh +0,50 rad/s. Dịch chuyển tâm cuối khoảng 0,42 mm; không buộc GT giữ nguyên x,y một cách toán học. Sai lệch yaw cuối odom-GT khoảng +0,4465°, cho thấy b hiệu dụng vẫn chỉ là hiệu chuẩn hữu hạn.'),
      'pivot_cw':('Kiểm tra dấu và tính đối xứng','Tốc độ quay ổn định khoảng -0,49755 rad/s. Hai bánh quay ngược chiều và IMU gyro z đổi dấu tương ứng. Dịch tâm cuối khoảng 0,49 mm và sai lệch yaw có dấu đối xứng gần với ca quay trái.'),
      'arc_left':('Ghép tiến và quay','Vận tốc ổn định khoảng 0,1500 m/s; ω ≈ 0,29854 rad/s, nên bán kính ổn định v/ω ≈ 0,5024 m. Quỹ đạo tổng còn gồm quá độ. Sai lệch vị trí cuối odom-GT ≈17,15 mm; không thể đánh giá chất lượng odometry chỉ bằng ca chạy thẳng.'),
      'arc_right':('Kiểm chứng cung đối xứng','Khi ω đổi dấu, đường đi và gyro đổi dấu gần đối xứng theo trục dọc. Sai lệch cuối odom-GT ≈17,12 mm. Độ lệch nhỏ giữa trái/phải có thể đến từ hình học COM, tiếp xúc và thời điểm lấy mẫu, chưa được tách nguyên nhân bằng thí nghiệm ablation.'),
      'brake':('Dừng từ vận tốc làm việc 0,30 m/s','Quãng dừng trung bình 100,78 mm, gần dự đoán v²/(2a) = 100 mm với a = 0,45 m/s². Phép thử phát zero chủ động, không phải thử mất truyền thông/watchdog hay emergency stop. Không dùng con số này như khoảng cách an toàn được chứng nhận.')}
    for i,(case,label) in enumerate(LABEL.items(),1):
        sm=SM[case];tr=next(x for x in TRIALS if x['case']==case)
        r.new(f'11.{i}. {label}: đáp ứng chuyển động','11 / HỒ SƠ THỬ',chapter=i==1)
        r.p(f'Lệnh: v = {num(tr["v"],2)} m/s; ω = {num(tr["w"],2)} rad/s; giữ {num(tr["nominal_duration"],0)} s; ba lần lặp. Đường màu biểu diễn các lần lặp, nét đứt biểu diễn lệnh; vạch đứng là mốc phát zero.','small')
        r.image('motion_'+case,f'[E1] {label}: vận tốc tuyến tính, vận tốc góc, góc quay tích lũy và đường đi trong hệ đầu lượt. Vận tốc suy từ ground truth, không lấy trực tiếp giá trị command.',345)
        r.table(['Đại lượng','Trung bình (n = 3)','SD mẫu'],[
            ['Quãng đường (m)',num(sm['path_length_m']['mean'],6),f"{sm['path_length_m']['sd']:.3g}"],
            ['Góc quay cuối (rad)',num(sm['yaw_change_rad']['mean'],6),f"{sm['yaw_change_rad']['sd']:.3g}"],
            ['v ổn định (m/s)',num(sm['steady_v_mps']['mean'],5),f"{sm['steady_v_mps']['sd']:.3g}"],
            ['ω ổn định (rad/s)',num(sm['steady_w_radps']['mean'],5),f"{sm['steady_w_radps']['sd']:.3g}"]],[.43,.32,.25])
        r.note(*insights[case])
        r.new(f'11.{i}. {label}: odom và cảm biến','11 / HỒ SƠ THỬ')
        r.image('sensors_'+case,f'[E1] {label}: sai lệch odom-GT, vận tốc bánh từ /joint_states và gyro z từ /imu/data. Màu ứng với lần lặp; bánh trái nét liền, bánh phải nét đứt.',345)
        r.table(['Chỉ số cuối / dừng','Trung bình ± SD'],[
            ['Sai lệch vị trí odom-GT (mm)',num(sm['odom_endpoint_error_m']['mean']*1000,4)+' ± '+f"{sm['odom_endpoint_error_m']['sd']*1000:.3g}"],
            ['Sai lệch yaw GT-odom (°)',num(math.degrees(sm['odom_yaw_endpoint_error_rad']['mean']),5)+' ± '+f"{math.degrees(sm['odom_yaw_endpoint_error_rad']['sd']):.3g}"],
            ['Quãng dừng sau zero (mm)',num(sm['stopping_distance_m']['mean']*1000,3)+' ± '+num(sm['stopping_distance_m']['sd']*1000,3)],
            ['Thời gian đến ngưỡng dừng (s)',num(sm['stopping_time_s']['mean'],3)+' ± '+num(sm['stopping_time_s']['sd'],3)]],[.57,.43])
        r.note('Cách sử dụng cho nghiên cứu','Đối chiếu đồng thời command, joint velocity, ground truth và IMU để kiểm tra dấu, quá độ và độ lệch mô hình. Các đường gần chồng lên nhau là tính lặp của cùng cấu hình, không phải tập thử độc lập trên nhiều tải, mặt sàn hoặc robot.')
        r.p('Nguồn số liệu: motion/metrics.csv và các CSV thô với nhãn '+case+'_1, '+case+'_2, '+case+'_3. SD ở mức rất nhỏ được giữ dạng khoa học để không nhầm thành độ phân giải vật lý.','small')

    r.new('12. Tổng hợp và ý nghĩa thực nghiệm','12 / THẢO LUẬN',True)
    r.image('odom_summary','Trung bình sai lệch cuối qua ba lần lặp mỗi ca. Chạy cung tạo sai lệch vị trí odometry lớn hơn các ca tiến/lùi hoặc quay tại chỗ trong đúng cấu hình này.',230)
    r.table(['Ca thử','L (m)','Δyaw (rad)','e_odom (mm)'],[[LABEL[s['case']],num(s['path_length_m']['mean'],4),num(s['yaw_change_rad']['mean'],4),num(s['odom_endpoint_error_m']['mean']*1000,3)] for s in SUM],[.40,.20,.20,.20])
    r.note('Điều có thể kết luận','Mô hình có đáp ứng tiến, lùi, quay và ghép chuyển động phù hợp dấu lệnh; các luồng odometry, joint state và IMU được ghi đồng thời. Giới hạn gia tốc giải thích được quá độ đi/dừng. Hiệu chỉnh b trong Gazebo giảm sai khác yaw nhưng chưa làm mọi quỹ đạo trùng ground truth.')
    r.p('Không có thống kê so sánh với robot thật, không có sweep tải/mặt sàn/ma sát và không có bộ lặp seed nhiễu độc lập. Những dữ liệu này là kiểm tra mô hình và khả năng tái lập, không tự đủ để tuyên bố ưu thế khoa học của một thuật toán mới.','small')
    r.new('12.1. Dừng và bất đối xứng tiến/lùi','12 / THẢO LUẬN')
    r.image('braking_calibration','Trái: mô hình dừng giải tích và ví dụ thêm độ trễ giả định. Phải: hai giá trị b trong mô tả mô phỏng. Không dùng các đường giải tích như phép đo thực.',245)
    r.table(['Tình huống','Dự đoán lý tưởng','Đo mô phỏng trung bình'],[['Tiến 0,20 m/s → zero','0,20² / (2 × 0,45) = 44,44 mm','44,76 mm'],['Lùi -0,20 m/s → zero','0,20² / (2 × 0,35) = 57,14 mm','57,46 mm'],['Tiến 0,30 m/s → zero','0,30² / (2 × 0,45) = 100,00 mm','100,78 mm']],[.37,.35,.28])
    r.p('Sai khác nhỏ còn lại chịu ảnh hưởng bước vật lý, khoảng phát command khoảng 51 ms, timestamp dừng và nội suy ground truth. Không coi sai khác này tự động là độ trễ cảm biến hay độ trễ điều khiển phần cứng.')
    r.p('Biểu đồ ví dụ trễ 0,25 s dùng s = vτ + v²/(2a) để cho thấy ảnh hưởng của trễ. 0,25 s là giả định minh họa theo profile, không phải thời gian mất lệnh đã được đo; thử nghiệm E1 không kiểm tra timeout.')
    r.new('12.2. Những khoảng trống cần công bố','12 / THẢO LUẬN')
    r.table(['Vấn đề','Mức ưu tiên','Tác động tới bài báo / triển khai'],[
        ['STEP b = 230,8; URDF b = 254,8 mm','Cao','Chốt revision cơ khí và đo b thật trước khi tuyên bố digital twin chính xác.'],
        ['Mặt hở STEP làm đổi bao lốp','Cao','Phân biệt bán kính nominal, bao CAD và bán kính lăn đo dưới tải.'],
        ['b hiệu dụng 283,4 mm','Cao','Chỉ có ý nghĩa trong mô hình tiếp xúc đã hiệu chỉnh; chưa được xác nhận trên mọi điều kiện.'],
        ['Profile hardware còn trường null','Cao','Encoder, driver, giao tiếp và bảo vệ phần cứng chưa đủ để tái lập robot thật.'],
        ['Collision đơn giản hóa theo chiều cao','Cao','Không công bố khả năng tránh mọi vật thấp, vượt gờ hoặc qua dưới vật treo từ mô hình 2D.'],
        ['Mass/inertia khai báo','Trung bình','Đo/cân hoặc trích mass CAD có material đáng tin trước khi phân tích tải và động lực học sâu.'],
        ['Mỗi ca chỉ 3 lặp cùng cấu hình','Trung bình','Không thay thế phép thử nhiều tải, sàn, seed nhiễu và độ trễ; chưa có power analysis.'],
        ['Một lượt kho dùng ThetaStar','Giới hạn phạm vi','Là minh họa tích hợp; không là benchmark PSTMO hay so sánh nhiều planner.']],[.36,.16,.48])
    r.p('Trong công việc này, không sửa STEP, URDF, SDF, thông số điều khiển hoặc dữ liệu nghiên cứu cũ. Chỉ bổ sung báo cáo, mã thu/đo/dựng hình và các dữ liệu thực nghiệm mới.','small')
    r.new('13. Cách dùng dữ liệu cho bài báo','13 / SỬ DỤNG',True)
    r.note('Một cấu trúc phần phương pháp có thể sử dụng','Mô tả nền tảng robot và hình học → phân biệt mô hình kinematic/visual/collision → công bố thông số cảm biến và vật lý → mô tả quy trình đo → định nghĩa chỉ số → trình bày kết quả → giới hạn. Đặt phát hiện khác biệt STEP/URDF trước các kết quả mô phỏng để người đọc biết chính xác nền tảng nào đã được kiểm tra.')
    r.table(['Nhóm hình / dữ liệu','Dùng để chứng minh','Không dùng để chứng minh'],[
        ['CAD đa góc và cấu trúc','Cấu tạo và vị trí theo tệp CAD','Thiết bị thực đã được lắp hoặc bền chắc'],['URDF/SDF/TF/collision','Cách hệ thống mô tả robot','Độ chính xác digital twin so với vật thật'],['Ảnh Gazebo và RViz2','Mô hình và các lớp ROS đã chạy','Chất lượng đường đi chỉ từ cảm giác nhìn đẹp'],['21 phép thử có dữ liệu thô','Đáp ứng và sai lệch mô phỏng theo protocol','Ưu thế thuật toán hoặc độ chính xác hardware'],['Lượt Nav2 trong kho','Tích hợp mô hình với bản đồ và navigation','Tỷ lệ thành công thống kê hoặc tối ưu đường']],[.32,.34,.34])
    r.note('Đoạn mô tả mẫu, không thay cho phần kết quả đầy đủ','“Nền tảng mô phỏng là robot vi sai với footprint 0,44 × 0,34 m, bán kính bánh danh định 0,0425 m và khối lượng khai báo 5,0 kg. URDF/SDF sử dụng khoảng cách bánh hình học 0,2548 m; plugin DiffDrive dùng khoảng cách hiệu dụng 0,2834 m. Hai giá trị này được tách biệt với khoảng cách tâm phần lốp 0,2308 m đo từ tệp STEP. Bảy phép thử động học, mỗi phép lặp ba lần, được ghi bằng thời gian mô phỏng và kiểm tra bằng ground truth độc lập với odometry bánh.”')
    r.p('Nếu lấy số liệu nghiên cứu năm quỹ đạo trước đây, phải trích đúng dữ liệu và protocol của báo cáo đó. Không gộp 21 lượt ở đây vào 125 lượt benchmark cũ, vì khác mục tiêu và khác cách điều khiển.','small')

    r.new('A. Tham số để tái lập','PHỤ LỤC',True)
    r.table(['Trường','Giá trị / nguồn'],[
        ['ROS / simulator','ROS 2 Jazzy; Gazebo Sim 8.15.0; GUI ghi backend dartsim, collision detector ode'],['Đồ họa CAD','cadquery-ocp 8.0.1.0.0; VTK 9.6.2; false colors; ảnh PNG 1.800 × 1.300'],['Thời gian vật lý','max_step_size = 0,003 s; real_time_factor danh định = 1,0'],['Domain E1 / E2','183 / 184; partition robot3d_report / robot3d_warehouse'],['World E1 / E2','open_arena / warehouse_cross_aisles'],['Publish command E1','Khoảng 20 Hz theo simulation clock, thực tế median ≈19,61 Hz'],['Pose reset E1','Model (-3; -2; 0,002 m), yaw = 0; settle 1,5 s'],['Giới hạn DiffDrive','v ±0,30 m/s; ω ±0,80 rad/s; a ∈ [-0,45; +0,35] m/s²; α ±1,20 rad/s²'],['Bánh / joint','r 0,0425 m; b_geom 0,2548 m; b_eff 0,2834 m; damping 0,03'],['Kiểm thử','5 passed: test_robot_description.py'],['Commit workspace',PROV['git_commit']]],[.38,.62])
    r.p('File STEP là dữ liệu người dùng bổ sung trong file_3D; mã hash lưu tại provenance.json. Commit không tự chứa mọi tệp untracked, nên phải đi kèm hash và bản nguồn. Ngày trong header STEP khác ngày ghi thí nghiệm và được giữ nguyên.')
    r.new('A.1. Lệnh và thứ tự tái tạo','PHỤ LỤC')
    r.note('1. Trích xuất và dựng hình','Dùng môi trường Python có numpy&lt;2, cadquery-ocp, vtk, trimesh, scipy, matplotlib, reportlab, pymupdf và PyYAML. Chạy lần lượt tools/robot3d_extract.py; robot3d_step_topology.py; robot3d_extract.py --render-only; robot3d_cad_audit.py. Bước topology bổ sung các mặt không nằm trong solid.')
    r.note('2. Ghi dữ liệu mô phỏng cơ bản','Source /opt/ros/jazzy/setup.bash và install/setup.bash. Đặt ROS_DOMAIN_ID=183, GZ_PARTITION=robot3d_report, PYTHONNOUSERSITE=1. Launch simulation.launch.py với environment:=open_arena, gui:=true, rviz:=false, nav2:=false, compare:=false, execute:=false; chọn spawn trong vùng trống (-3; -2). Chạy tools/robot3d_motion_test.py.')
    r.note('3. Thu ảnh đúng ứng dụng','tools/robot3d_capture.py gazebo chụp camera quanh vị trí (-3; -2); tools/robot3d_capture.py rviz tạo tám cấu hình góc nhìn. Không chạy đồng thời hai bộ capture: cửa sổ bị che có thể làm X11 trả ảnh lỗi. Trước khi chụp Gazebo phải đặt lại robot tại tâm camera và phát zero.')
    r.note('4. Minh họa kho','Phiên riêng domain 184, partition robot3d_warehouse; launch warehouse_cross_aisles với nav2:=true, compare:=false, execute:=false, spawn (-5; 0). tools/robot3d_warehouse.py chọn ThetaStar bằng /planner_selector rồi gửi goal (5; 0). Dừng các tiến trình đúng phiên đã tạo sau khi thu xong.')
    r.note('5. Tính, dàn trang và kiểm tra','Chạy tools/robot3d_analysis.py; tools/robot3d_supplement.py; tools/build_robot3d_report.py; tools/verify_robot3d_report.py. Khi chạy lại bộ thu, dùng bản sao thư mục assets hoặc sao lưu CSV trước, vì script ghi tệp kết quả ổn định theo tên.')
    r.p('Không chạy các lệnh thử trực tiếp trên hệ ROS của robot thật. Script motion_test có kiểm tra domain/partition cố định; các node này chỉ được dùng cùng Gazebo đã khởi tạo cho báo cáo.','small')

    r.new('B. Hồ sơ solid STEP quan trọng','PHỤ LỤC',True)
    r.p('ID dưới đây là thứ tự khi duyệt solid bằng OCCT, không phải part number. Đơn vị theo mm, mm³. Danh sách đầy đủ 542 solid có trong step_solids.csv; 168 PRODUCT record có trong cad_product_records.txt.','small')
    rows=[]
    for s in sorted(G['step']['solids'],key=lambda x:-x['volume_mm3'])[:18]:
        b=s['bounds_mm'];dims=np.array(b[3:])-b[:3]
        rows.append([str(s['id']),num(s['volume_mm3'],1),' × '.join(num(v,2) for v in dims),'; '.join(num(x,2) for x in s['centroid_mm'])])
    r.table(['ID','Volume mm³','Bao X × Y × Z gốc (mm)','Tâm thể tích X;Y;Z'],rows,[.08,.20,.37,.35])
    r.p('Tâm thể tích với mật độ đồng nhất không phải COM thật của một vật có nhiều vật liệu. Các mặt ngoài solid không có thể tích kín để cộng vào bảng này.','small')
    r.new('B.1. Ma trận và nguồn gốc thông số','PHỤ LỤC')
    r.table(['Inertia tại COM riêng (kg·m²)','Ixx','Iyy','Izz'],[['base_link','0,042710097','0,107573321','0,137292434'],['left_wheel','0,000123851','0,000221573','0,000123851'],['right_wheel','0,000123851','0,000221573','0,000123851']],[.34,.22,.22,.22])
    r.p('Off-diagonal thân: Ixy = 0,000327650; Ixz = -0,000920746; Iyz = 0,000111809 kg·m². Off-diagonal hai bánh bằng 0. Rpy inertial bằng 0, nên cộng tensor trong cùng hệ trục bằng định lý trục song song.')
    r.note('Công thức tổng hợp được dùng','M = Σmᵢ; c = Σmᵢcᵢ/M.<br/>I_COM = Σ[Iᵢ + mᵢ((dᵢ·dᵢ)E - dᵢdᵢᵀ)], với dᵢ = cᵢ - c.<br/>Các tọa độ cᵢ đã cộng pose link theo SDF; không cộng offset visual STL vào COM.')
    r.table(['Nhận định','Nguồn trực tiếp'],[['Kích thước solid và free face','Xe.step → OCCT; geometry.json + step_topology.json'],['Khoảng cách bánh STEP','Hộp bao solid #27/#34; cad_audit.json'],['Cấu trúc link/joint/visual','vacuum_robot.urdf; geometry.json'],['Khối lượng, inertia, sensor, DiffDrive','model.sdf; mass_properties.json'],['Giới hạn điều khiển và profile','nav2_params.yaml; real_robot_profile.yaml'],['Số liệu mô phỏng','motion/*.csv; trials.json; warehouse_demo*.json'],['Nguồn từng hình','figure_page_index.json; capture_manifest.jsonl']],[.43,.57])

    metrics=list(csv.DictReader((A/'motion/metrics.csv').open()))
    for part,rows in enumerate([metrics[:11],metrics[11:]],1):
        r.new(f'C.{part}. Bảng số liệu của 21 lượt','PHỤ LỤC',chapter=part==1)
        r.p('L gồm baseline và toàn bộ pha đi/dừng của lượt; e là sai lệch odom-GT ở cuối, sau khi căn hệ đầu lượt. Các số làm tròn phục vụ đọc; CSV giữ giá trị đầy đủ.','small')
        r.table(['Lượt','L (m)','Δyaw (rad)','e (mm)','s dừng (mm)','t dừng (s)'],[[m['trial'],num(float(m['path_length_m']),5),num(float(m['yaw_change_rad']),5),num(float(m['odom_endpoint_error_m'])*1000,4),num(float(m['stopping_distance_m'])*1000,3),num(float(m['stopping_time_s']),3)] for m in rows],[.25,.15,.17,.15,.15,.13])
        r.note('Các cột bổ sung trong CSV','metrics.csv còn chứa endpoint_x_m, endpoint_y_m, vận tốc ổn định, yaw-error có dấu, RMSE vị trí và tốc độ ngang cực đại. CSV thô giúp đổi vùng tính ổn định hoặc ngưỡng dừng mà không chạy lại simulator.')
        r.p('Ba lần lặp không đổi khối lượng, ma sát, cấu hình cảm biến hoặc initial yaw. Không sử dụng SD rất nhỏ để ước lượng sai số phần cứng; độ lặp của simulator có thể gần tất định.')

    r.new('D. Nguồn, tệp bàn giao và kiểm chứng','PHỤ LỤC',True)
    r.table(['Đường dẫn trong docs/robot_3d_report','Nội dung'],[['figures/','Ảnh dựng CAD/STL, sơ đồ kỹ thuật, đồ thị thực nghiệm'],['screenshots/','PNG gốc chụp cửa sổ Gazebo và RViz2'],['motion/','6 CSV luồng dữ liệu; trials, metrics, summary, rates; scan snapshot'],['rviz/','Các cấu hình góc nhìn và lớp hiển thị để chụp lại'],['geometry.json / step_solids.csv','Hộp bao, thể tích, diện tích, centroid, số tam giác từng solid'],['cad_audit.json / step_topology.json','Sai khác bánh và kiểm tra các mặt nằm ngoài solid'],['warehouse_demo*.json','Dữ liệu lượt abort ban đầu và lượt chạy thành công sau chọn planner'],['provenance.json','Phiên bản môi trường, commit, hash các nguồn'],['figure_page_index.json','Nối số hình, trang PDF và tệp ảnh nguồn'],['logs/ / qa/','Log RViz, kiểm thử hợp đồng, ảnh render và báo cáo kiểm tra PDF']],[.47,.53])
    r.p('Mọi ảnh kỹ thuật trong báo cáo được dựng từ tệp nguồn hoặc dữ liệu thực nghiệm; không dùng ảnh tạo sinh để thay ảnh Gazebo/RViz2. Các ảnh chụp giữ giao diện; chúng không ghi toàn màn hình cá nhân của người dùng.')
    r.note('Kết luận sử dụng','Bộ dữ liệu đủ làm hồ sơ kỹ thuật nền tảng mô phỏng và đầu vào cho phân tích chuyển động. Phát hiện chưa đồng nhất STEP/URDF cần được giải quyết trước khi gọi hệ thống là bản sao số đã kiểm chứng của robot thật. Không có sửa đổi tự động vào cấu hình nguồn trong quá trình lập báo cáo.')
    # Selected authoritative hashes, full list is machine-readable alongside PDF.
    r.new('D.1. Dấu vân tay các tệp nguồn chính','PHỤ LỤC')
    selected=['file_3D/Xe.step','src/vacuum_robot_gazebo/urdf/vacuum_robot.urdf','src/vacuum_robot_gazebo/models/vacuum_robot/model.sdf','src/vacuum_robot_gazebo/config/nav2_params.yaml','src/vacuum_robot_gazebo/config/real_robot_profile.yaml','src/vacuum_robot_gazebo/worlds/open_arena.sdf','src/vacuum_robot_gazebo/worlds/warehouse_cross_aisles.sdf']
    for p in selected:
        src=next(s for s in PROV['sources'] if s['path']==p)
        r.p(e(p),'h2');h=src['sha256'];r.p('SHA-256: '+h[:32]+'<br/>'+h[32:]+'<br/>'+str(src['bytes'])+' byte','small')
    r.p('Hash định danh chính xác nội dung tệp tại thời điểm lập báo cáo; không chứng minh tính đúng của mô hình. Khi cập nhật cơ khí/cấu hình, cần sinh lại dữ liệu và báo cáo phù hợp revision.','small')
    # Figure list: compact, clickable, limited to clear one-line titles.
    figs=list(r.figures)
    for part,start in enumerate(range(0,len(figs),23),1):
        r.new(f'E.{part}. Danh mục hình và trang tra cứu','PHỤ LỤC',chapter=part==1)
        rows=[]
        for f in figs[start:start+23]:
            short=f['caption'].split('. ')[0]
            if len(short)>105:short=short[:102]+'...'
            rows.append([str(f['figure']),short,str(f['page'])])
        r.table(['Hình','Nội dung rút gọn','Trang'],rows,[.09,.81,.10])
        r.p('Tên PNG đầy đủ và chú thích nguyên văn nằm trong figure_page_index.json. Ảnh gốc có độ phân giải cao hơn kích thước hiển thị trong PDF, phù hợp trích xuất lại cho bản thảo.','small')

def main():
    temp=Path('/tmp/robot3d-report-firstpass.pdf');r=Report(temp);author(r);index=r.finish()
    final=Report(PDF,total=len(index['pages']),prior=index);author(final);idx=final.finish()
    assert len(idx['pages'])==len(index['pages'])
    (A/'figure_page_index.json').write_text(json.dumps(idx,ensure_ascii=False,indent=2))
    print(json.dumps(dict(pdf=str(PDF),pages=len(idx['pages']),figures=len(idx['figures']),bytes=PDF.stat().st_size)))
if __name__=='__main__':main()
