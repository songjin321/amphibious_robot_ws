using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace Intel.RealSense
{
    public class DeviceList : IDisposable, IEnumerable<Device>
    {
        IntPtr m_instance;

        public DeviceList(IntPtr ptr)
        {
            m_instance = ptr;
        }


        public IEnumerator<Device> GetEnumerator()
        {
            object error;

            int deviceCount = NativeMethods.rs2_get_device_count(m_instance, out error);
            for (int i = 0; i < deviceCount; i++)
            {
                var ptr = NativeMethods.rs2_create_device(m_instance, i, out error);
                yield return new Device(ptr);
            }
        }

        IEnumerator IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }

        public int Count
        {
            get
            {
                object error;
                int deviceCount = NativeMethods.rs2_get_device_count(m_instance, out error);
                return deviceCount;
            }
        }

        public Device this[int index]
        {
            get
            {
                object error;
                var ptr = NativeMethods.rs2_create_device(m_instance, index, out error);
                return new Device(ptr);
            }
        }

        public bool Contains(Device device)
        {
            object error;
            return System.Convert.ToBoolean(NativeMethods.rs2_device_list_contains(m_instance, device.m_instance, out error));
        }

        #region IDisposable Support
        private bool disposedValue = false; // To detect redundant calls

        protected virtual void Dispose(bool disposing)
        {
            if (!disposedValue)
            {
                if (disposing)
                {
                    // TODO: dispose managed state (managed objects).
                }

                // TODO: free unmanaged resources (unmanaged objects) and override a finalizer below.
                // TODO: set large fields to null.
                NativeMethods.rs2_delete_device_list(m_instance);
                m_instance = IntPtr.Zero;

                disposedValue = true;
            }
        }

        //TODO: override a finalizer only if Dispose(bool disposing) above has code to free unmanaged resources.
        ~DeviceList()
        {
            // Do not change this code. Put cleanup code in Dispose(bool disposing) above.
            Dispose(false);
        }

        // This code added to correctly implement the disposable pattern.
        public void Dispose()
        {
            // Do not change this code. Put cleanup code in Dispose(bool disposing) above.
            Dispose(true);
            // TODO: uncomment the following line if the finalizer is overridden above.
            GC.SuppressFinalize(this);
        }
        #endregion
    }
}
