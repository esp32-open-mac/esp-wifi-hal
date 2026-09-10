//! Reproduce the observed automatic echo loss with no Wi-Fi driver or hardware.

#[cfg(test)]
mod tests {
    use smoltcp::{
        iface::{Config, Interface, SocketSet},
        phy::{ChecksumCapabilities, Device, DeviceCapabilities, Medium, RxToken, TxToken},
        time::Instant,
        wire::{
            ArpOperation, ArpPacket, ArpRepr, EthernetAddress, EthernetFrame, EthernetProtocol,
            Icmpv4Packet, Icmpv4Repr, IpAddress, IpCidr, IpProtocol, Ipv4Address, Ipv4Packet,
            Ipv4Repr,
        },
    };
    use std::collections::VecDeque;

    const DEVICE_MAC: EthernetAddress = EthernetAddress([2, 0, 0, 0, 0, 1]);
    const HOST_MAC: EthernetAddress = EthernetAddress([2, 0, 0, 0, 0, 2]);
    const DEVICE_IP: Ipv4Address = Ipv4Address::new(192, 0, 2, 1);
    const HOST_IP: Ipv4Address = Ipv4Address::new(192, 0, 2, 2);

    #[derive(Default)]
    struct MockDevice {
        rx: VecDeque<Vec<u8>>,
        tx: Vec<Vec<u8>>,
    }
    struct Receive(Vec<u8>);
    struct Transmit<'a>(&'a mut Vec<Vec<u8>>);
    impl RxToken for Receive {
        fn consume<R, F: FnOnce(&[u8]) -> R>(self, f: F) -> R {
            f(&self.0)
        }
    }
    impl TxToken for Transmit<'_> {
        fn consume<R, F: FnOnce(&mut [u8]) -> R>(self, len: usize, f: F) -> R {
            let mut bytes = vec![0; len];
            let result = f(&mut bytes);
            self.0.push(bytes);
            result
        }
    }
    impl Device for MockDevice {
        type RxToken<'a> = Receive;
        type TxToken<'a> = Transmit<'a>;
        fn receive(&mut self, _: Instant) -> Option<(Receive, Transmit<'_>)> {
            self.rx
                .pop_front()
                .map(|bytes| (Receive(bytes), Transmit(&mut self.tx)))
        }
        fn transmit(&mut self, _: Instant) -> Option<Transmit<'_>> {
            Some(Transmit(&mut self.tx))
        }
        fn capabilities(&self) -> DeviceCapabilities {
            let mut caps = DeviceCapabilities::default();
            caps.medium = Medium::Ethernet;
            caps.max_transmission_unit = 1514;
            caps
        }
    }

    fn ethernet(protocol: EthernetProtocol, payload: usize) -> Vec<u8> {
        let mut bytes = vec![0; 14 + payload];
        let mut frame = EthernetFrame::new_unchecked(&mut bytes[..]);
        frame.set_src_addr(HOST_MAC);
        frame.set_dst_addr(DEVICE_MAC);
        frame.set_ethertype(protocol);
        bytes
    }

    fn echo(sequence: u16) -> Vec<u8> {
        let checksum = ChecksumCapabilities::default();
        let repr = Icmpv4Repr::EchoRequest {
            ident: 42,
            seq_no: sequence,
            data: &[0x5a; 512],
        };
        let ip = Ipv4Repr {
            src_addr: HOST_IP,
            dst_addr: DEVICE_IP,
            next_header: IpProtocol::Icmp,
            payload_len: repr.buffer_len(),
            hop_limit: 64,
        };
        let mut bytes = ethernet(EthernetProtocol::Ipv4, ip.buffer_len() + repr.buffer_len());
        ip.emit(&mut Ipv4Packet::new_unchecked(&mut bytes[14..]), &checksum);
        repr.emit(
            &mut Icmpv4Packet::new_unchecked(&mut bytes[34..]),
            &checksum,
        );
        bytes
    }

    fn arp_reply() -> Vec<u8> {
        let repr = ArpRepr::EthernetIpv4 {
            operation: ArpOperation::Reply,
            source_hardware_addr: HOST_MAC,
            source_protocol_addr: HOST_IP,
            target_hardware_addr: DEVICE_MAC,
            target_protocol_addr: DEVICE_IP,
        };
        let mut bytes = ethernet(EthernetProtocol::Arp, repr.buffer_len());
        repr.emit(&mut ArpPacket::new_unchecked(&mut bytes[14..]));
        bytes
    }

    #[test]
    fn automatic_echo_reply_is_lost_during_arp_then_next_echo_succeeds() {
        let mut device = MockDevice::default();
        let mut iface = Interface::new(Config::new(DEVICE_MAC.into()), &mut device, Instant::ZERO);
        iface.update_ip_addrs(|addrs| {
            addrs
                .push(IpCidr::new(IpAddress::Ipv4(DEVICE_IP), 24))
                .unwrap()
        });
        let mut sockets = SocketSet::new(vec![]);

        device.rx.push_back(echo(1));
        iface.poll(Instant::from_millis(1), &mut device, &mut sockets);
        assert_eq!(device.tx.len(), 1);
        let arp = ArpPacket::new_checked(&device.tx[0][14..]).unwrap();
        assert_eq!(arp.operation(), ArpOperation::Request);
        device.tx.clear();

        device.rx.push_back(arp_reply());
        iface.poll(Instant::from_millis(2), &mut device, &mut sockets);
        iface.poll(Instant::from_millis(100), &mut device, &mut sockets);
        assert!(
            device.tx.is_empty(),
            "The first echo reply was not queued for retry"
        );

        device.rx.push_back(echo(2));
        iface.poll(Instant::from_millis(200), &mut device, &mut sockets);
        assert_eq!(device.tx.len(), 1);
        let ip = Ipv4Packet::new_checked(&device.tx[0][14..]).unwrap();
        let packet = Icmpv4Packet::new_checked(ip.payload()).unwrap();
        assert!(matches!(
            Icmpv4Repr::parse(&packet, &ChecksumCapabilities::default()),
            Ok(Icmpv4Repr::EchoReply {
                ident: 42,
                seq_no: 2,
                ..
            })
        ));
    }
}
