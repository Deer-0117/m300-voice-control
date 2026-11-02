"""Linux descriptor datagrams. All received descriptors have one close owner."""
import array
import os
import socket
import stat
import struct

from . import protocol as p


def _read_fd(fd, limit, expected=None):
    import fcntl
    info=os.fstat(fd)
    if not stat.S_ISREG(info.st_mode) or not 0 < info.st_size <= limit:
        raise ValueError('descriptor type/size')
    if fcntl.fcntl(fd,fcntl.F_GETFL) & os.O_ACCMODE != os.O_RDONLY:
        raise ValueError('descriptor must be read-only')
    if expected is not None and info.st_size != expected:
        raise ValueError('metadata size mismatch')
    chunks=[]; offset=0
    while offset < info.st_size:
        chunk=os.pread(fd,min(65536,info.st_size-offset),offset)
        if not chunk: raise ValueError('descriptor shortened during read')
        chunks.append(chunk); offset+=len(chunk)
    after=os.fstat(fd)
    if (after.st_size,after.st_mtime_ns,after.st_ctime_ns)!=(info.st_size,info.st_mtime_ns,info.st_ctime_ns):
        raise ValueError('descriptor modified during read')
    return b''.join(chunks)


def receive_message(sock, allowed_uid:int, allowed_gid:int):
    """Return (validated header with payload, image bytes), or None on EAGAIN.

    The bound receiver MUST set SO_PASSCRED before accepting producer traffic.
    Credentials are required here even if the sender includes identity JSON.
    """
    fds=[]
    try:
        try:
            raw, ancillary, flags, _ = sock.recvmsg(p.MAX_WIRE_BYTES,
                socket.CMSG_SPACE(256 * array.array('i').itemsize) + socket.CMSG_SPACE(12),
                getattr(socket,'MSG_CMSG_CLOEXEC',0) | getattr(socket,'MSG_DONTWAIT',0))
        except BlockingIOError:
            return None
        credentials=[]; bad_ancillary=False
        for level,kind,data in ancillary:
            if level==socket.SOL_SOCKET and kind==socket.SCM_RIGHTS:
                values=array.array('i')
                usable=len(data)-(len(data)%values.itemsize)
                values.frombytes(data[:usable]); fds.extend(values)
                bad_ancillary |= usable != len(data)
            elif level==socket.SOL_SOCKET and kind==socket.SCM_CREDENTIALS:
                if len(data)==12: credentials.append(struct.unpack('3i',data))
                else: bad_ancillary=True
            else:
                bad_ancillary=True
        if bad_ancillary or flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC):
            raise ValueError('truncated or invalid ancillary data')
        if len(credentials)!=1 or credentials[0][1:]!=(allowed_uid,allowed_gid):
            raise ValueError('sender credentials')
        for fd in fds: os.set_inheritable(fd,False)
        header=p.loads(raw); p.validate_header(header)
        expected=1 if header['metadata_mode']=='inline' else 2
        if len(fds)!=expected: raise ValueError('descriptor count')
        payload=(header['payload'] if expected==1 else
                 p.loads(_read_fd(fds[1],p.MAX_METADATA_BYTES,header['metadata_bytes']),p.MAX_METADATA_BYTES))
        p.validate_payload(header,payload)
        image=_read_fd(fds[0],p.MAX_IMAGE_BYTES)
        return {**header,'payload':payload}, image
    finally:
        for fd in fds:
            try: os.close(fd)
            except OSError: pass
