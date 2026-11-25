# Simscape Robot Conveyor Belts

## 1. Giới thiệu

Dự án mô phỏng hệ thống robot youBot 6 bậc tự do kết hợp băng tải và cơ cấu gắp sản phẩm bằng MATLAB, Simulink và Simscape.

Hệ thống mô phỏng quy trình:
1. Băng tải đầu vào đưa hộp đến vị trí làm việc.
2. Cảm biến Light Curtain phát hiện hộp.
3. Băng tải đầu vào dừng.
4. Robot di chuyển đến vị trí lấy hộp.
5. Kẹp robot đóng để giữ hộp.
6. Robot nâng, xoay và chuyển hộp.
7. Băng tải đầu ra vận chuyển hộp khỏi khu vực làm việc.

## 2. Công nghệ

- MATLAB R2024a.
- Simulink.
- Simscape và các thành phần cần thiết để mở mô hình.
- MATLAB scripts và dữ liệu MAT.

## 3. Cấu trúc thư mục

```text
Simscape-Robot-Conveyor-Belts/
|-- CAD/
|-- Images/
|-- Libraries/
|-- Overview/
|-- Scripts_Data/
|-- Workflows/
|   |-- Optimize/
|-- results/
|   |-- models/
|   |-- trajectory_dashboard/
|   |-- trajectory_methods/
|-- youBot_Arm.slx
|-- README.md
```

## 4. Cài đặt và chuẩn bị

1. Cài MATLAB R2024a.
2. Cài Simulink và các sản phẩm Simscape mà mô hình yêu cầu.
3. Tải hoặc clone repository về máy.
4. Giải nén dự án nếu tải dưới dạng ZIP.
5. Mở MATLAB.
6. Chuyển Current Folder đến thư mục gốc dự án.

Repository:

https://github.com/huy-bit/Simscape-Robot-Conveyor-Belts

## 5. Khởi tạo dự án

Mở MATLAB Command Window và chạy:

```matlab
root = 'D:\Simscape-Robot-Conveyor-Belts-24.1.2.7';
cd(root);
run(fullfile(root, 'Scripts_Data', 'startup_youBot.m'));
```

Script startup_youBot.m dùng để chuẩn bị môi trường làm việc của dự án.

## 6. Chạy mô hình robot

Mở mô hình chính:

```matlab
cd('D:\Simscape-Robot-Conveyor-Belts-24.1.2.7');
open_system('youBot_Arm');
```

Trong cửa sổ Simulink:

1. Kiểm tra mô hình đã mở đầy đủ.
2. Kiểm tra các biến và dữ liệu đầu vào đã được khởi tạo.
3. Nhấn nút Run trên thanh công cụ Simulink.
4. Theo dõi chuyển động robot, băng tải và các tín hiệu trong mô hình.

Chạy mô phỏng bằng MATLAB:

```matlab
mdl = 'youBot_Arm';
load_system(mdl);
sim(mdl);
```

Có thể chạy script demo nếu cần:

```matlab
run(fullfile(pwd, 'Scripts_Data', 'youBot_Arm_Demo_Script.m'));
```

## 7. Các chế độ điều khiển

Dự án có các hướng cấu hình đầu vào và điều khiển:

- Control: cấu hình điều khiển.
- Signals: điều khiển bằng tín hiệu.
- Splines: điều khiển quỹ đạo chuyển động bằng spline.
- Control Belt: điều khiển băng tải vòng kín và cánh tay robot vòng hở.

Các chế độ phụ thuộc cấu hình của mô hình và dữ liệu đầu vào tương ứng.

## 8. Phân tích 12 phương pháp quỹ đạo

Dự án có bộ phân tích quỹ đạo gồm 12 phương pháp:

1. Linear.
2. Trapezoidal.
3. LSPB.
4. Cubic Polynomial.
5. Quintic Polynomial.
6. Septic Polynomial.
7. Sinusoidal.
8. Cycloidal.
9. S-Curve.
10. Bezier.
11. Minimum Jerk.
12. Minimum Snap.

### Mở Trajectory Dashboard

Để mở giao diện Trajectory Dashboard, thực hiện các bước sau:

**Bước 1.** Mở MATLAB.

**Bước 2.** Nhập đường dẫn thư mục dự án:

```matlab
root = 'D:\Simscape-Robot-Conveyor-Belts-24.1.2.7';
cd(root);
```

**Bước 3.** Chạy lệnh mở Dashboard:

```matlab
run(fullfile(root, ...
    'results', ...
    'trajectory_dashboard', ...
    'trajectory_dashboard.m'));
```

**Bước 4.** MATLAB mở giao diện Trajectory Dashboard.

Sau khi giao diện xuất hiện, chọn phương pháp quỹ đạo cần phân tích và thao tác trên Dashboard.

### Các đoạn chuyển động

- Home2In: từ vị trí Home đến vị trí lấy hộp.
- In2Out: từ vị trí đầu vào đến vị trí đầu ra.
- Out2Home: từ vị trí đầu ra trở về Home.

## 9. Dữ liệu và kết quả

Kết quả mô phỏng được tổ chức trong thư mục results.

| Thư mục | Nội dung |
|---|---|
| results/models | Các mô hình Simulink và bản lưu mô hình |
| results/trajectory_dashboard | Dashboard và script tạo dữ liệu quỹ đạo |
| results/trajectory_methods | Dữ liệu và kết quả theo từng phương pháp |

Các định dạng dữ liệu được dự án sử dụng gồm:

- MAT: dữ liệu MATLAB.
- CSV: dữ liệu dạng bảng.
- PNG: hình ảnh kết quả.
- FIG: biểu đồ MATLAB có thể mở và chỉnh sửa.

## 10. Mở kết quả

Mở file MAT:

```matlab
load('results/trajectory_methods/trajectory_12_methods.mat');
```

Mở hình PNG:

```matlab
imshow('results/trajectory_methods/example.png');
```

Mở biểu đồ FIG:

```matlab
openfig('results/trajectory_methods/example.fig');
```

Tên file cụ thể phụ thuộc kết quả đã được tạo trong thư mục dự án.

## 11. Một số lỗi thường gặp

### Không tìm thấy biến hoặc tham số

- Kiểm tra Current Folder.
- Chạy script khởi tạo startup_youBot.m.
- Kiểm tra dữ liệu MAT và các script cấu hình.

### Không mở được mô hình

- Kiểm tra file youBot_Arm.slx.
- Kiểm tra phiên bản MATLAB và các sản phẩm Simulink, Simscape cần thiết.
- Kiểm tra đường dẫn Libraries.

### Mô phỏng báo lỗi block hoặc thư viện

- Kiểm tra các thư viện được cài đặt.
- Kiểm tra MATLAB path.
- Kiểm tra các file phụ thuộc của mô hình.

### Cảnh báo model bị shadowed

Kiểm tra MATLAB path để tránh có nhiều file cùng tên youBot_Arm.slx. Ưu tiên mô hình nằm trong thư mục dự án hiện tại.

## 12. Tài liệu tham khảo

- MathWorks Simulink: https://www.mathworks.com/help/simulink/
- MathWorks Simscape: https://www.mathworks.com/help/simscape/
- GitHub repository: https://github.com/huy-bit/Simscape-Robot-Conveyor-Belts

## 13. Tác giả và đóng góp

Repository: huy-bit/Simscape-Robot-Conveyor-Belts

Project: Simscape Robot Conveyor Belts.
