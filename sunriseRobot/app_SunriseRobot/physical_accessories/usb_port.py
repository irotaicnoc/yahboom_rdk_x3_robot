import usb.core
import usb.util


def find_usb_device_port(vendor_id, product_id, device_name=""):
    """Finds the USB device and tries to determine its port."""
    try:
        dev = usb.core.find(idVendor=vendor_id, idProduct=product_id)

        if dev is None:
            print(f"Device with VID:PID {vendor_id:04x}:{product_id:04x} not found.")
            return None

        timeout = 100000
        version = dev.ctrl_transfer(
            usb.util.CTRL_IN | usb.util.CTRL_TYPE_VENDOR | usb.util.CTRL_RECIPIENT_DEVICE,
            0, 0x80, 0, 1, timeout)[0]
        print('version: {}'.format(version))

        bus = dev.bus
        address = dev.address
        try:
            import subprocess
            result = subprocess.run(
                ["udevadm", "info", "--attribute-walk", "--name", f"/dev/bus/usb/{bus:03d}/{address:03d}"],
                capture_output=True, text=True, check=True
            )
            for line in result.stdout.splitlines():
                if "ATTRS{port}==" in line:
                    port = line.split("==")[1].strip('"')
                    print(f"Likely connected to USB port: {port}")
                    return port
        except FileNotFoundError:
            print("udevadm not found.")
        except subprocess.CalledProcessError as e:
            print(f"Error running udevadm: {e}")

        print("Could not reliably determine the USB port.")
        return None

    except usb.core.USBError as e:
        print(f"USB error: {e}")
        return None


if __name__ == "__main__":
    vendor_id = 0x2886
    product_id = 0x0018
    device_name = "ReSpeaker 4 Mic Array"

    find_usb_device_port(vendor_id, product_id, device_name)