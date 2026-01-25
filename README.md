# Data Flow Overview

Luồng dữ liệu từ lúc gửi lệnh điều khiển cho tới khi đọc lại trạng thái actuator được mô tả như sau:

SetMitCmd()
→ QueueCommand()
→ SPI Transfer
→ RX Frame
→ PushRxData()
→ OnDataReceived()
→ ParseFeedback()
→ Update position_ / velocity_ / torque_
→ GetPosition() returns updated value

powershell
Sao chép mã

### Mô tả các bước

- **SetMitCmd()**  
  Tạo lệnh điều khiển actuator (MIT mode).

- **QueueCommand()**  
  Đưa lệnh vào hàng đợi để chuẩn bị gửi qua SPI.

- **SPI Transfer**  
  Thực hiện truyền dữ liệu qua SPI bus.

- **RX Frame**  
  Nhận frame phản hồi từ actuator.

- **PushRxData() / OnDataReceived()**  
  Đẩy dữ liệu nhận được vào pipeline xử lý.

- **ParseFeedback()**  
  Parse dữ liệu phản hồi: position, velocity, torque.

- **Update position_ / velocity_ / torque_**  
  Cập nhật trạng thái nội bộ.

- **GetPosition()**  
  Trả về giá trị position mới nhất.

---

# SPI Configuration Note (Radxa Rock 5B Plus)

> **Important:**  
> Khi sử dụng **Radxa Rock 5B Plus**, nếu muốn dùng **CS0 và CS1 đồng thời trên SPI0**,  
> **không thể bật hai overlay riêng biệt** do xung đột GPIO.
>
> Giải pháp đúng là tạo **một device-tree overlay duy nhất** kết hợp cả CS0 và CS1.

---

## Combined SPI0 CS0 + CS1 Overlay

### Device Tree Source  
**File:** `rk3588-spi0-m2-cs0-cs1.dts`

```dts
/dts-v1/;
/plugin/;

/ {
	metadata {
		title = "Enable spidev on SPI0-M2 over CS0 and CS1";
		compatible = "radxa,rock-5a",
		             "radxa,rock-5b",
		             "radxa,rock-5b-plus",
		             "radxa,rock-5c",
		             "radxa,nx5-io",
		             "radxa,cm5-rpi-cm4-io";
		category = "misc";
		exclusive = "GPIO1_B3", "GPIO1_B2", "GPIO1_B1",
		            "GPIO1_B4", "GPIO1_B5";
		description = "Enable spidev on SPI0-M2 over CS0 and CS1.";
	};
};

&spi0 {
	status = "okay";
	#address-cells = <1>;
	#size-cells = <0>;

	pinctrl-names = "default";
	pinctrl-0 = <&spi0m2_cs0 &spi0m2_cs1 &spi0m2_pins>;

	max-freq = <50000000>;

	spidev@0 {
		compatible = "rockchip,spidev";
		status = "okay";
		reg = <0>;
		spi-max-frequency = <50000000>;
	};

	spidev@1 {
		compatible = "rockchip,spidev";
		status = "okay";
		reg = <1>;
		spi-max-frequency = <50000000>;
	};
};
```
Build & Install Overlay
1. Compile overlay
bash
Sao chép mã
dtc -@ -I dts -O dtb \
  -o rk3588-spi0-m2-cs0-cs1.dtbo \
  rk3588-spi0-m2-cs0-cs1.dts
2. Copy overlay to system directory
bash
Sao chép mã
sudo cp rk3588-spi0-m2-cs0-cs1.dtbo \
  /boot/dtbs/rockchip/overlay/
3. Configure bootloader
Chỉnh sửa một trong các file sau (tùy distro):

/boot/extlinux/extlinux.conf

/boot/armbianEnv.txt

Thêm dòng:

ini
Sao chép mã
fdtoverlays=/boot/dtbs/rockchip/overlay/rk3588-spi0-m2-cs0-cs1.dtbo
4. Reboot
bash
Sao chép mã
sudo reboot
Verification
Sau khi reboot, kiểm tra các SPI devices:

bash
Sao chép mã
ls /dev/spidev*
Kết quả mong đợi:

bash
Sao chép mã
/dev/spidev0.0   # CS0
/dev/spidev0.1   # CS1
Summary
SPI0 hỗ trợ nhiều chip select.

Không bật nhiều overlay CS riêng lẻ (sẽ gây conflict GPIO).

Cách đúng: một overlay duy nhất khai báo đầy đủ CS0 và CS1.