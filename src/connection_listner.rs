use ecore::connection::Connection;
use std::{
    marker::PhantomData,
    net::{TcpListener, ToSocketAddrs},
};

pub struct ConnectionListner<const N: usize, T> {
    listner: TcpListener,
    _marker: PhantomData<T>,
}

impl<const N: usize, T> ConnectionListner<N, T> {
    pub fn bind<A: ToSocketAddrs>(addr: A) -> std::io::Result<Self> {
        Ok(Self {
            listner: TcpListener::bind(addr)?,
            _marker: PhantomData,
        })
    }

    pub fn accept(&self) -> std::io::Result<Connection<N, T>> {
        Ok(Connection::new(self.listner.accept()?.0))
    }
}
