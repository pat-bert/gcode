class ApiException(Exception):
    pass


class MelfaError(ApiException):
    pass


class GCmdError(ApiException):
    pass


class TcpError(ApiException):
    pass


class PreCheckError(ApiException):
    pass


class OutOfOperationalArea(PreCheckError):
    pass
