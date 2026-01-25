Thay vì sử dụng hai overlay riêng biệt, bạn cần tạo một overlay mới kết hợp cả hai chip select:
dts/dts-v1/;
/plugin/;

/ {
	metadata {
		title = "Enable spidev on SPI0-M2 over CS0 and CS1";
		compatible = "radxa,rock-5a", "radxa,rock-5b", "radxa,rock-5b-plus", "radxa,rock-5c", "radxa,nx5-io", "radxa,cm5-rpi-cm4-io";
		category = "misc";
		exclusive = "GPIO1_B3", "GPIO1_B2", "GPIO1_B1", "GPIO1_B4", "GPIO1_B5";
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
Các bước thực hiện:

Tạo file overlay mới (ví dụ: rk3588-spi0-m2-cs0-cs1.dts)
Biên dịch overlay:

bashdtc -@ -I dts -O dtb -o rk3588-spi0-m2-cs0-cs1.dtbo rk3588-spi0-m2-cs0-cs1.dts

Copy file .dtbo vào thư mục overlay:

bashsudo cp rk3588-spi0-m2-cs0-cs1.dtbo /boot/dtbs/rockchip/overlay/
```

4. **Chỉnh sửa file cấu hình** (thường là `/boot/extlinux/extlinux.conf` hoặc `/boot/armbianEnv.txt`):
```
fdtoverlays=/boot/dtbs/rockchip/overlay/rk3588-spi0-m2-cs0-cs1.dtbo

Reboot hệ thống

Kiểm tra sau khi reboot:
bashls /dev/spidev*
Bạn sẽ thấy cả hai device:

/dev/spidev0.0 (CS0)
/dev/spidev0.1 (CS1)